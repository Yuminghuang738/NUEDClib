"""
Train an ultra-light CNN on MNIST and export TFLite for OpenMV (4.8.1 compatible).

Usage (Colab — TensorFlow is pre-installed):
    Just paste and run. Uses concrete-function export for max compatibility.

Usage (PC):
    pip install tensorflow
    python train_num_model.py

Architecture (lightweight — ~16 K params):
    ┌──────────────────────────────────────────────────┐
    │  Input 28×28×1                                   │
    │    ↓   (data augmentation — training only)       │
    │  RandomRotation(±15°) + Translation(±10%) + Zoom │
    │    ↓                                             │
    │  Conv3×3×8  → MaxPool2×2  → 14×14×8              │
    │    ↓                                             │
    │  Conv3×3×16 → MaxPool2×2  → 7×7×16              │
    │    ↓                                             │
    │  Conv3×3×32 → MaxPool2×2  → 3×3×32              │
    │    ↓                                             │
    │  Flatten → Dense32(ReLU) → Dropout0.3 → Dense10  │
    └──────────────────────────────────────────────────┘

Output:
    num_identify.tflite  →  copy to OpenMV SD card root
"""

import tensorflow as tf
import numpy as np
import os

# ── Constants ──
IMG_SIZE   = 28
BATCH      = 64
EPOCHS     = 30
MODEL_NAME = "num_identify"

# ── Load & preprocess MNIST ──
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()

x_train = x_train.astype("float32") / 255.0
x_test  = x_test.astype("float32")  / 255.0

# Add channel dim: (N, 28, 28) → (N, 28, 28, 1)
x_train = np.expand_dims(x_train, -1)
x_test  = np.expand_dims(x_test,  -1)

y_train = tf.keras.utils.to_categorical(y_train, 10)
y_test  = tf.keras.utils.to_categorical(y_test,  10)

# ── Build model ──
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(IMG_SIZE, IMG_SIZE, 1)),

    # Data augmentation — active only during training.
    # Bridges the gap between clean MNIST digits and noisy camera frames.
    tf.keras.layers.RandomRotation(0.15, fill_mode="constant", fill_value=0),
    tf.keras.layers.RandomTranslation(0.10, 0.10, fill_mode="constant", fill_value=0),
    tf.keras.layers.RandomZoom(0.15, fill_mode="constant", fill_value=0),

    # Block 1
    tf.keras.layers.Conv2D(8, 3, padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(2),               # 28 → 14

    # Block 2
    tf.keras.layers.Conv2D(16, 3, padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(2),               # 14 → 7

    # Block 3
    tf.keras.layers.Conv2D(32, 3, padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(2),               # 7 → 3

    tf.keras.layers.Flatten(),                     # 3×3×32 = 288

    tf.keras.layers.Dense(32, activation="relu"),
    tf.keras.layers.Dropout(0.3),
    tf.keras.layers.Dense(10, activation="softmax"),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss="categorical_crossentropy",
    metrics=["accuracy"],
)

model.summary()

# ── Callbacks ──
callbacks = [
    tf.keras.callbacks.ReduceLROnPlateau(
        monitor="val_loss", factor=0.5, patience=3, min_lr=1e-5, verbose=1
    ),
    tf.keras.callbacks.EarlyStopping(
        monitor="val_accuracy", patience=8, restore_best_weights=True, verbose=1
    ),
]

# ── Train ──
model.fit(x_train, y_train,
          batch_size=BATCH,
          epochs=EPOCHS,
          validation_data=(x_test, y_test),
          callbacks=callbacks)

# ── Evaluate ──
loss, acc = model.evaluate(x_test, y_test, verbose=0)
print(f"\nTest accuracy: {acc:.4f}  ({acc*100:.1f} %)")

# ── Export to TFLite via concrete-function (most compatible path for older runtimes) ──
@tf.function(input_signature=[tf.TensorSpec(shape=[1, IMG_SIZE, IMG_SIZE, 1],
                                            dtype=tf.float32)])
def infer(x):
    return model(x, training=False)   # disables dropout & augmentation


concrete_func = infer.get_concrete_function()
converter = tf.lite.TFLiteConverter.from_concrete_functions([concrete_func])
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
converter.optimizations = []          # no quantisation — pure float32, most compatible

tflite_model = converter.convert()

tflite_path = f"{MODEL_NAME}.tflite"
with open(tflite_path, "wb") as f:
    f.write(tflite_model)

file_kb = len(tflite_model) / 1024
print(f"\nModel exported: {tflite_path}  ({len(tflite_model)} bytes / {file_kb:.1f} KB)")
print("Copy this file to the OpenMV SD card root directory.")
