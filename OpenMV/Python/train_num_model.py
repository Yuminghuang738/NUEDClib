"""
Train an ultra-light CNN on MNIST and export TFLite for OpenMV (4.8.1 compatible).

Usage (Colab — TensorFlow is pre-installed):
    Just paste and run. Uses concrete-function export for max compatibility.

Usage (PC):
    pip install tensorflow
    python train_num_model.py

Architecture (tiny — ~7K params):
    Conv3x3x4 → MaxPool2x2 → Conv3x3x8 → MaxPool2x2 → Flatten
    → Dense16(ReLU) → Dropout0.2 → Dense10(Softmax)

Output:
    num_identify.tflite  →  copy to OpenMV SD card root
"""

import tensorflow as tf
import numpy as np

# ── Constants ──
IMG_SIZE    = 28
BATCH       = 64
EPOCHS      = 15
MODEL_NAME  = "num_identify"

# ── Load & preprocess MNIST ──
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.mnist.load_data()

x_train = x_train.astype("float32") / 255.0
x_test  = x_test.astype("float32")  / 255.0

# Add channel dim: (N, 28, 28) → (N, 28, 28, 1)
x_train = np.expand_dims(x_train, -1)
x_test  = np.expand_dims(x_test,  -1)

y_train = tf.keras.utils.to_categorical(y_train, 10)
y_test  = tf.keras.utils.to_categorical(y_test,  10)

# ── Build ultra-light model ──
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(IMG_SIZE, IMG_SIZE, 1)),

    tf.keras.layers.Conv2D(4, 3, padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(2),

    tf.keras.layers.Conv2D(8, 3, padding="same", activation="relu"),
    tf.keras.layers.MaxPooling2D(2),

    tf.keras.layers.Flatten(),

    tf.keras.layers.Dense(16, activation="relu"),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(10, activation="softmax"),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss="categorical_crossentropy",
    metrics=["accuracy"],
)

model.summary()

# ── Train ──
model.fit(x_train, y_train, batch_size=BATCH, epochs=EPOCHS,
          validation_data=(x_test, y_test))

# ── Evaluate ──
loss, acc = model.evaluate(x_test, y_test, verbose=0)
print(f"\nTest accuracy: {acc:.4f}")

# ── Export to TFLite via concrete-function (most compatible path for older runtimes) ──
@tf.function(input_signature=[tf.TensorSpec(shape=[1, IMG_SIZE, IMG_SIZE, 1], dtype=tf.float32)])
def infer(x):
    return model(x, training=False)

concrete_func = infer.get_concrete_function()
converter = tf.lite.TFLiteConverter.from_concrete_functions([concrete_func])
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
converter.optimizations = []  # no quantisation — pure float32, most compatible

tflite_model = converter.convert()

tflite_path = f"{MODEL_NAME}.tflite"
with open(tflite_path, "wb") as f:
    f.write(tflite_model)

print(f"\nModel exported: {tflite_path}  ({len(tflite_model)} bytes)")
print("Copy this file to the OpenMV SD card root directory.")
