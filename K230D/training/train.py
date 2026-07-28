"""
钢球数量识别 —— 训练脚本。

在 PC 上训练 CNN 分类模型，预测图像中的钢球数量 (0~MAX_COUNT)。

使用方法:
    # 1. 先生成数据集
    cd dataset/
    python generate_synthetic.py --total 5000 --max-count 15 --show-samples
    cd ..

    # 2. 训练模型
    python training/train.py --model small --epochs 50 --batch 32

    # 3. 推理验证
    python training/train.py --eval-only --weights models/steelball_small.h5
"""

import os
import sys
import argparse

import numpy as np
import cv2
import tensorflow as tf
from tensorflow import keras
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score

# 添加父目录到 path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from training.model import create_model, MAX_COUNT, IMG_SIZE, CHANNELS


# ── 默认路径 ──
DEFAULT_DATASET_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset"
)
DEFAULT_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models"
)


# ═══════════════════════════════════════════════════════════════════
# 数据加载
# ═══════════════════════════════════════════════════════════════════

def load_dataset(dataset_dir: str, img_size: int = IMG_SIZE, num_classes: int = None):
    """从目录加载数据集。

    期望目录结构:
        dataset/
        ├── train/    # 训练图像
        ├── val/      # 验证图像
        └── labels.csv (可选)

    文件名格式: {count}_{id}.jpg  例如  03_00001.jpg → count=3

    Returns:
        (x_train, y_train), (x_val, y_val): 归一化的图像和 one-hot 标签
    """
    def load_split(split_dir, n_classes):
        if not os.path.isdir(split_dir):
            print(f"Warning: {split_dir} not found, returning empty")
            return np.array([]), np.array([])

        images, labels = [], []
        files = [f for f in os.listdir(split_dir) if f.endswith(('.jpg', '.png', '.bmp'))]

        for fname in files:
            # 解析 count: 文件名格式 {count}_{id}.jpg 或 extra_{id}_{count}.jpg
            parts = fname.replace(".jpg", "").replace(".png", "").replace(".bmp", "").split("_")
            if parts[0].isdigit():
                count = int(parts[0])
            elif parts[0] == "extra":
                count = int(parts[-1])
            else:
                continue

            if n_classes and count >= n_classes:
                continue

            # 读取灰度图
            fpath = os.path.join(split_dir, fname)
            img = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            img = cv2.resize(img, (img_size, img_size))
            img = img.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=-1)  # (H,W) → (H,W,1)

            images.append(img)
            labels.append(count)

        x = np.array(images, dtype=np.float32)
        y = tf.keras.utils.to_categorical(labels, n_classes)
        return x, y

    # Auto-detect num_classes from data if not specified
    if num_classes is None:
        train_dir = os.path.join(dataset_dir, "train")
        if os.path.isdir(train_dir):
            max_label = 0
            for fname in os.listdir(train_dir):
                if fname.endswith(('.jpg', '.png', '.bmp')):
                    parts = fname.replace(".jpg", "").replace(".png", "").replace(".bmp", "").split("_")
                    if parts[0].isdigit():
                        max_label = max(max_label, int(parts[0]))
                    elif parts[0] == "extra":
                        max_label = max(max_label, int(parts[-1]))
            num_classes = max_label + 1
        else:
            num_classes = MAX_COUNT + 1

    print(f"  Detected {num_classes} classes (0 ~ {num_classes - 1})")

    train_dir = os.path.join(dataset_dir, "train")
    val_dir   = os.path.join(dataset_dir, "val")

    print(f"Loading training data from {train_dir} ...")
    x_train, y_train = load_split(train_dir, num_classes)
    print(f"  Loaded {len(x_train)} training samples")

    print(f"Loading validation data from {val_dir} ...")
    x_val, y_val = load_split(val_dir, num_classes)
    print(f"  Loaded {len(x_val)} validation samples")

    return (x_train, y_train), (x_val, y_val)


# ═══════════════════════════════════════════════════════════════════
# 数据增强
# ═══════════════════════════════════════════════════════════════════

def create_data_augmentation():
    """创建训练时的数据增强 pipeline。

    只在训练时应用，模拟真实场景的变化:
      - 随机旋转 (±30°)
      - 随机平移 (±15%)
      - 随机缩放 (±15%)
      - 随机亮度变化
      - 水平/垂直翻转 (球的位置应对称)
    """
    return keras.Sequential([
        keras.layers.RandomRotation(0.15, fill_mode="constant", fill_value=0),
        keras.layers.RandomTranslation(0.15, 0.15, fill_mode="constant", fill_value=0),
        keras.layers.RandomZoom(0.15, fill_mode="constant", fill_value=0),
        keras.layers.RandomFlip("horizontal"),
        keras.layers.RandomFlip("vertical"),
        keras.layers.RandomBrightness(0.15, value_range=(0, 1)),
    ], name="augmentation")


# ═══════════════════════════════════════════════════════════════════
# 训练主流程
# ═══════════════════════════════════════════════════════════════════

def train(args):
    # ── 加载数据 ──
    dataset_dir = args.dataset_dir or DEFAULT_DATASET_DIR
    num_classes = args.num_classes  # may be None, will be auto-detected
    (x_train, y_train), (x_val, y_val) = load_dataset(dataset_dir, args.img_size, num_classes)

    if len(x_train) == 0:
        print("❌ No training data found! Run generate_synthetic.py first.")
        sys.exit(1)

    # Use detected num_classes
    num_classes = y_train.shape[1]
    y_train_int = np.argmax(y_train, axis=1)
    class_counts = np.bincount(y_train_int, minlength=num_classes).astype(np.float32)
    class_counts[class_counts == 0] = 1  # 避免除零
    class_weight = {}
    for i in range(num_classes):
        if class_counts[i] > 0:
            class_weight[i] = len(y_train) / (num_classes * class_counts[i])

    # ── 构建模型 ──
    input_shape = (args.img_size, args.img_size, CHANNELS)
    model = create_model(args.model, num_classes=num_classes, input_shape=input_shape)
    model.summary()

    # 编译
    optimizer = keras.optimizers.Adam(learning_rate=args.lr)
    model.compile(
        optimizer=optimizer,
        loss="categorical_crossentropy",
        metrics=["accuracy"],
    )

    # ── 回调 ──
    callbacks = [
        keras.callbacks.ModelCheckpoint(
            filepath=os.path.join(args.models_dir, f"steelball_{args.model}_best.h5"),
            monitor="val_accuracy",
            save_best_only=True,
            verbose=1,
        ),
        keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss", factor=0.5, patience=5, min_lr=1e-6, verbose=1,
        ),
        keras.callbacks.EarlyStopping(
            monitor="val_accuracy", patience=args.patience,
            restore_best_weights=True, verbose=1,
        ),
    ]

    # ── 训练 ──
    history = model.fit(
        x_train, y_train,
        batch_size=args.batch,
        epochs=args.epochs,
        validation_data=(x_val, y_val) if len(x_val) > 0 else None,
        callbacks=callbacks,
        class_weight=class_weight,
        verbose=1,
    )

    # ── 保存最终模型 ──
    final_path = os.path.join(args.models_dir, f"steelball_{args.model}.h5")
    model.save(final_path)
    print(f"\n✅ Model saved to: {final_path}")

    # ── 评估 ──
    if len(x_val) > 0:
        evaluate_model(model, x_val, y_val, args)

    # ── 导出 TFLite ──
    export_tflite(model, args)

    return model, history


# ═══════════════════════════════════════════════════════════════════
# 评估
# ═══════════════════════════════════════════════════════════════════

def evaluate_model(model, x_val, y_val, args):
    """在验证集上评估模型性能。"""
    y_true = np.argmax(y_val, axis=1)
    y_pred = np.argmax(model.predict(x_val, verbose=0), axis=1)

    acc = accuracy_score(y_true, y_pred)
    mae = np.mean(np.abs(y_true.astype(float) - y_pred.astype(float)))

    print()
    print("═" * 60)
    print("📊  Evaluation Results")
    print("═" * 60)
    print(f"  Accuracy (exact match): {acc:.4f} ({acc * 100:.1f}%)")
    print(f"  MAE (mean absolute error): {mae:.2f} balls")
    print(f"  Off-by-1 accuracy: {(np.abs(y_true - y_pred) <= 1).mean():.4f}")
    print()
    print("Classification Report:")
    present_classes = sorted(set(int(c) for c in np.unique(np.concatenate([y_true, y_pred]))))
    target_names = [str(i) for i in present_classes]
    labels = present_classes
    print(classification_report(y_true, y_pred, target_names=target_names,
                                 labels=labels, zero_division=0))


def eval_only(args):
    """仅评估已保存的模型。"""
    dataset_dir = args.dataset_dir or DEFAULT_DATASET_DIR
    _, (x_val, y_val) = load_dataset(dataset_dir, args.img_size)

    if len(x_val) == 0:
        print("❌ No validation data found!")
        sys.exit(1)

    model = keras.models.load_model(args.eval_weights)
    evaluate_model(model, x_val, y_val, args)


# ═══════════════════════════════════════════════════════════════════
# TFLite 导出 (兼容 K230D nncase)
# ═══════════════════════════════════════════════════════════════════

def export_tflite(model, args):
    """导出 TFLite 模型 (float32)。

    导出的 TFLite 可通过 nncase 工具链转换为 .kmodel 用于 K230D KPU。
    """
    import os as _os

    # 强制 CPU-only 避免 CUDA 库未安装导致的转换失败
    _os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

    # 使用 Keras 原生 TFLite 导出 (不经过 grappler GPU 优化器)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
    converter.optimizations = []  # float32, 兼容性最好

    tflite_model = converter.convert()

    tflite_path = os.path.join(args.models_dir, f"steelball_{args.model}.tflite")
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    size_kb = len(tflite_model) / 1024
    print(f"\n📦 TFLite model exported: {tflite_path}")
    print(f"   Size: {size_kb:.1f} KB  ({len(tflite_model):,} bytes)")
    print(f"   Next: Use nncase to convert to .kmodel for K230D KPU")


# ═══════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="钢球数量识别 — CNN 模型训练"
    )
    # 数据
    parser.add_argument("--dataset-dir", type=str, default=None,
                        help=f"数据集目录 (默认: {DEFAULT_DATASET_DIR})")
    parser.add_argument("--img-size", type=int, default=IMG_SIZE,
                        help=f"输入图像尺寸 (默认: {IMG_SIZE})")

    # 模型
    parser.add_argument("--model", type=str, default="small",
                        choices=["tiny", "small", "medium"],
                        help="模型大小 (默认: small)")
    parser.add_argument("--num-classes", type=int, default=None,
                        help=f"输出类别数 (默认: MAX_COUNT+1={MAX_COUNT+1})")

    # 训练
    parser.add_argument("--epochs", type=int, default=50,
                        help="训练轮数 (默认: 50)")
    parser.add_argument("--batch", type=int, default=32,
                        help="批量大小 (默认: 32)")
    parser.add_argument("--lr", type=float, default=0.001,
                        help="初始学习率 (默认: 0.001)")
    parser.add_argument("--patience", type=int, default=12,
                        help="早停耐心值 (默认: 12)")

    # 输出
    parser.add_argument("--models-dir", type=str, default=DEFAULT_MODELS_DIR,
                        help=f"模型输出目录 (默认: {DEFAULT_MODELS_DIR})")

    # 仅评估
    parser.add_argument("--eval-only", action="store_true",
                        help="仅评估已保存的模型")
    parser.add_argument("--eval-weights", type=str, default=None,
                        help="模型权重路径 (.h5)")

    args = parser.parse_args()

    os.makedirs(args.models_dir, exist_ok=True)

    if args.eval_only:
        if not args.eval_weights:
            print("❌ Please specify --eval-weights PATH")
            sys.exit(1)
        eval_only(args)
    else:
        train(args)


if __name__ == "__main__":
    main()
