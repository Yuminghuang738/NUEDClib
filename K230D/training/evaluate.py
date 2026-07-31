"""
钢球数量识别 —— 模型评估脚本。

对训练好的模型进行详细的性能评估:
  - 精确匹配率 (accuracy)
  - 平均绝对误差 (MAE)
  - ±1 容差准确率
  - 混淆矩阵可视化
  - 各类别 F1-score
  - 错误样本分析

用法:
    python training/evaluate.py --model models/steelball_small.h5
    python training/evaluate.py --model models/steelball_small.h5 --plot
"""

import os
import sys
import argparse
import json

import numpy as np
import cv2
import tensorflow as tf
from tensorflow import keras
from sklearn.metrics import (
    classification_report,
    confusion_matrix,
    accuracy_score,
    mean_absolute_error,
)

# 添加父目录
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from training.model import MAX_COUNT, IMG_SIZE


def load_validation_data(dataset_dir: str, img_size: int = IMG_SIZE):
    """加载验证集。"""
    val_dir = os.path.join(dataset_dir, "val")
    if not os.path.isdir(val_dir):
        val_dir = os.path.join(dataset_dir, "train")  # fallback

    images, labels = [], []
    files = [f for f in os.listdir(val_dir) if f.endswith(('.jpg', '.png', '.bmp'))]

    for fname in files:
        parts = fname.replace(".jpg", "").replace(".png", "").replace(".bmp", "").split("_")
        if parts[0].isdigit():
            count = int(parts[0])
        elif parts[0] == "extra":
            count = int(parts[-1])
        else:
            continue
        if count > MAX_COUNT:
            continue

        fpath = os.path.join(val_dir, fname)
        img = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        img = cv2.resize(img, (img_size, img_size))
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=-1)
        images.append(img)
        labels.append(count)

    x = np.array(images, dtype=np.float32)
    y = np.array(labels, dtype=np.int32)
    return x, y


def evaluate(model_path: str, dataset_dir: str, img_size: int, plot: bool = False):
    """全面评估模型。"""

    print("=" * 60)
    print("Steel Ball Counting — Model Evaluation")
    print("=" * 60)

    # 加载模型
    print(f"\nLoading model: {model_path}")
    model = keras.models.load_model(model_path)

    # 加载数据
    default_dataset = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset"
    )
    dataset_dir = dataset_dir or default_dataset
    print(f"Loading data from: {dataset_dir}")
    x, y_true = load_validation_data(dataset_dir, img_size)
    print(f"  {len(x)} validation samples")

    if len(x) == 0:
        print("No validation data found!")
        return

    # 推理
    print("Running inference...")
    y_pred_probs = model.predict(x, batch_size=32, verbose=1)
    y_pred = np.argmax(y_pred_probs, axis=1)

    # ── 指标计算 ──
    acc = accuracy_score(y_true, y_pred)
    mae = mean_absolute_error(y_true.astype(float), y_pred.astype(float))
    off_by_1 = (np.abs(y_true - y_pred) <= 1).mean()
    off_by_2 = (np.abs(y_true - y_pred) <= 2).mean()

    print()
    print("─" * 40)
    print("📊  Overall Metrics")
    print("─" * 40)
    print(f"  Accuracy (exact):        {acc:.4f}  ({acc*100:.1f}%)")
    print(f"  MAE (mean abs error):    {mae:.3f} balls")
    print(f"  Within ±1 ball:          {off_by_1:.4f}  ({off_by_1*100:.1f}%)")
    print(f"  Within ±2 balls:         {off_by_2:.4f}  ({off_by_2*100:.1f}%)")
    print()

    # ── 各类别统计 ──
    print("─" * 40)
    print("📋  Per-Class Accuracy")
    print("─" * 40)
    print(f"  {'Count':>6s}  {'Samples':>8s}  {'Correct':>8s}  {'Acc':>8s}  {'MAE':>8s}")
    print(f"  {'─'*6}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
    per_class = {}
    for c in sorted(set(y_true)):
        mask = (y_true == c)
        n = mask.sum()
        correct = (y_pred[mask] == c).sum()
        class_acc = correct / n if n > 0 else 0
        class_mae = np.abs(y_true[mask].astype(float) - y_pred[mask].astype(float)).mean()
        per_class[int(c)] = {"n": int(n), "correct": int(correct), "acc": float(class_acc), "mae": float(class_mae)}
        print(f"  {c:>6d}  {n:>8d}  {correct:>8d}  {class_acc:>7.3f}  {class_mae:>7.3f}")

    # ── 错误分析 ──
    errors = y_true != y_pred
    n_errors = errors.sum()
    print()
    print(f"  Total errors: {n_errors}/{len(y_true)} ({n_errors/len(y_true)*100:.1f}%)")

    # Top 错误模式
    if n_errors > 0:
        print()
        print("  Top error patterns (true → predicted):")
        error_pairs = {}
        for t, p in zip(y_true[errors], y_pred[errors]):
            key = f"{t}→{p}"
            error_pairs[key] = error_pairs.get(key, 0) + 1
        for pattern, count in sorted(error_pairs.items(), key=lambda x: -x[1])[:10]:
            print(f"    {pattern}: {count} times")

    # ── 保存结果 ──
    results = {
        "model": model_path,
        "num_samples": len(x),
        "accuracy": float(acc),
        "mae": float(mae),
        "within_1": float(off_by_1),
        "within_2": float(off_by_2),
        "per_class": per_class,
    }
    result_path = model_path.replace(".h5", "_eval.json")
    with open(result_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to: {result_path}")

    # ── 混淆矩阵图 ──
    if plot:
        _plot_confusion(y_true, y_pred, model_path)


def _plot_confusion(y_true, y_pred, model_path):
    """绘制混淆矩阵。"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    cm = confusion_matrix(y_true, y_pred)
    # 归一化 (行)
    cm_norm = cm.astype(float) / (cm.sum(axis=1, keepdims=True) + 1e-8)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # 原始计数
    im1 = ax1.imshow(cm, cmap="Blues", aspect="auto")
    ax1.set_title("Confusion Matrix (counts)", fontsize=14)
    ax1.set_xlabel("Predicted")
    ax1.set_ylabel("True")
    plt.colorbar(im1, ax=ax1)

    # 归一化
    im2 = ax2.imshow(cm_norm, cmap="YlOrRd", aspect="auto", vmin=0, vmax=1)
    ax2.set_title("Confusion Matrix (normalized by row)", fontsize=14)
    ax2.set_xlabel("Predicted")
    ax2.set_ylabel("True")
    plt.colorbar(im2, ax=ax2)

    plt.tight_layout()
    plot_path = model_path.replace(".h5", "_confusion.png")
    plt.savefig(plot_path, dpi=120)
    plt.show()
    print(f"Confusion matrix plot saved to: {plot_path}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate steel ball counting model")
    parser.add_argument("--model", type=str, required=True, help="Path to .h5 model file")
    parser.add_argument("--dataset-dir", type=str, default=None,
                        help="Dataset directory (default: ../dataset/)")
    parser.add_argument("--img-size", type=int, default=IMG_SIZE,
                        help=f"Input image size (default: {IMG_SIZE})")
    parser.add_argument("--plot", action="store_true", help="Plot confusion matrix")
    args = parser.parse_args()

    evaluate(args.model, args.dataset_dir, args.img_size, args.plot)


if __name__ == "__main__":
    main()
