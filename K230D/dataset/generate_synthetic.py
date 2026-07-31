"""
生成钢球合成数据集 —— 用于训练钢球数量识别神经网络。

原理:
    在随机背景上绘制带金属质感的圆形（模拟钢球），
    每张图像有 0~MAX_COUNT 个不重叠的钢球。
    同时添加光照变化、噪点、轻微运动模糊等数据增强，
    使模型能泛化到真实场景。

输出:
    dataset/
    ├── train/
    │   ├── 00001_3.jpg   # 文件名格式: {id}_{count}.jpg
    │   ├── 00002_7.jpg
    │   └── ...
    ├── val/
    │   └── ...
    └── labels.csv        # filename,count  (方便查阅)

用法:
    python generate_synthetic.py                     # 默认参数
    python generate_synthetic.py --total 5000        # 自定义数量
    python generate_synthetic.py --max-count 20      # 最大球数
    python generate_synthetic.py --show-samples      # 生成并显示样例
"""

import os
import sys
import math
import random
import argparse
import csv

import cv2
import numpy as np
from tqdm import tqdm

# ── 可调参数 ──
IMG_SIZE      = 224           # 输出图像尺寸 (正方形, 224×224 适配 K230D KPU 常见输入)
MAX_COUNT     = 15            # 单张图像中最大的钢球数量
TOTAL_SAMPLES = 3000          # 总样本数 (训练集)
VAL_SAMPLES   = 500           # 验证集样本数
BALL_RADIUS_MIN = 12          # 钢球最小半径 (像素)
BALL_RADIUS_MAX = 25          # 钢球最大半径 (像素)
NOISE_LEVEL   = 8             # 高斯噪声标准差

# 输出目录
OUT_DIR       = "dataset"
TRAIN_DIR     = os.path.join(OUT_DIR, "train")
VAL_DIR       = os.path.join(OUT_DIR, "val")


# ═══════════════════════════════════════════════════════════════════
# 钢球纹理生成
# ═══════════════════════════════════════════════════════════════════

def make_ball_texture(radius: int) -> np.ndarray:
    """生成一个带金属光泽的圆形纹理 (灰度图)。

    模拟钢球的外观:
      - 基底为深灰色 (金属色)
      - 左上方有明亮的高光 (specular highlight)
      - 右下边缘较暗
      - 整体呈现球形立体感

    Returns:
        np.ndarray: shape (2R, 2R), dtype uint8, 球在 Alpha 区域
    """
    d = 2 * radius
    # 坐标网格
    y, x = np.ogrid[-radius:radius, -radius:radius]
    dist = np.sqrt(x * x + y * y)                  # 到球心的距离
    inside = (dist <= radius)

    # 归一化坐标 [-1, 1]
    nx = x / (radius + 1e-6)
    ny = y / (radius + 1e-6)

    # 基础金属色 ~100 (中灰)
    base = np.full((d, d), 100.0, dtype=np.float32)

    # 高光: 模拟光源从左上方照射
    # 光方向: (-0.6, -0.6, 0.5) 归一化 → 左上方向
    light_x, light_y, light_z = -0.55, -0.55, 0.65
    nz = np.sqrt(np.clip(1.0 - nx * nx - ny * ny, 0, 1))  # 球面法线 Z 分量
    nx3d = nx.copy()
    ny3d = ny.copy()
    # 点积: n·l
    diffuse = nx3d * light_x + ny3d * light_y + nz * light_z
    diffuse = np.clip(diffuse, 0, 1)

    # 镜面高光 (Blinn-Phong 近似)
    half_x, half_y, half_z = 0.0, 0.0, 1.0            # 半角向量 (视角 = 正上方)
    specular = nx3d * half_x + ny3d * half_y + nz * half_z
    specular = np.clip(specular, 0, 1) ** 8            # 高光锐度

    # 合成
    brightness = base + 80 * diffuse + 80 * specular
    brightness = np.clip(brightness, 0, 255)

    texture = np.where(inside, brightness, 0).astype(np.uint8)
    return texture


# ═══════════════════════════════════════════════════════════════════
# 背景生成
# ═══════════════════════════════════════════════════════════════════

def make_background(size: int) -> np.ndarray:
    """生成随机的暗色背景。

    包括:
      - 纯暗色表面 (模拟黑布/黑色泡沫)
      - 带轻微纹理的表面
      - 带渐变光照的表面
    """
    bg_type = random.choice(["plain", "textured", "gradient", "noisy"])

    if bg_type == "plain":
        # 纯暗灰背景
        gray = random.randint(20, 60)
        bg = np.full((size, size), gray, dtype=np.uint8)

    elif bg_type == "textured":
        # 带细微纹理的背景 (如布料纹理)
        gray = random.randint(25, 55)
        bg = np.full((size, size), gray, dtype=np.float32)
        # 生成低频噪声
        tex_scale = random.randint(4, 8)
        small = size // tex_scale
        noise = np.random.randint(-12, 13, (small, small)).astype(np.float32)
        noise = cv2.resize(noise, (size, size), interpolation=cv2.INTER_LINEAR)
        bg = np.clip(bg + noise, 0, 255).astype(np.uint8)

    elif bg_type == "gradient":
        # 光照渐变 (模拟不均匀照明)
        bg = np.zeros((size, size), dtype=np.float32)
        cx = random.randint(0, size)
        cy = random.randint(0, size)
        y, x = np.ogrid[:size, :size]
        dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        gradient = 1.0 - 0.4 * dist / (size * 1.5)
        gradient = np.clip(gradient, 0.3, 1.0)
        base = random.randint(20, 50)
        bg = (base * gradient).astype(np.uint8)

    elif bg_type == "noisy":
        # 纯噪声背景
        gray = random.randint(20, 50)
        bg = np.full((size, size), gray, dtype=np.float32)
        noise = np.random.randint(-15, 16, (size, size)).astype(np.float32)
        bg = np.clip(bg + noise, 0, 255).astype(np.uint8)

    return bg


# ═══════════════════════════════════════════════════════════════════
# 主生成函数
# ═══════════════════════════════════════════════════════════════════

def generate_image(count: int, size: int = IMG_SIZE) -> np.ndarray:
    """生成一张包含 count 个钢球的灰度图像。

    钢球随机放置，用拒绝采样避免重叠。
    """
    img = make_background(size).astype(np.float32)
    alpha_mask = np.zeros((size, size), dtype=np.float32)

    attempts = 0
    max_attempts = count * 50
    placed = 0

    while placed < count and attempts < max_attempts:
        attempts += 1

        radius = random.randint(BALL_RADIUS_MIN, BALL_RADIUS_MAX)
        # 随机位置 (留出边缘余量)
        margin = radius + 2
        cx = random.randint(margin, size - margin - 1)
        cy = random.randint(margin, size - margin - 1)

        # 检查重叠 (用 alpha_mask)
        diam = 2 * radius
        x0 = cx - radius
        y0 = cy - radius

        # 快速重叠检测: 检查该区域 alpha 通道是否已被占用
        roi = alpha_mask[y0:y0 + diam, x0:x0 + diam]
        if np.any(roi > 0):
            continue  # 重叠, 跳过

        # 生成钢球纹理
        texture = make_ball_texture(radius)

        # 亮度随机微调 (模拟不同钢球表面的微小差异)
        brightness_jitter = random.uniform(0.85, 1.15)
        texture = np.clip(texture.astype(np.float32) * brightness_jitter, 0, 255)

        # 贴入背景 (alpha 混合)
        texture_f = texture.astype(np.float32)
        bg_patch = img[y0:y0 + diam, x0:x0 + diam]
        # 钢球区域: 用纹理替换
        img[y0:y0 + diam, x0:x0 + diam] = np.where(
            texture > 5, texture_f, bg_patch
        )

        # 标记已占用区域
        alpha_mask[y0:y0 + diam, x0:x0 + diam] = np.where(
            texture > 5, 1.0, alpha_mask[y0:y0 + diam, x0:x0 + diam]
        )

        placed += 1

    # ── 后处理: 添加全局光照变化、噪点 ──
    img = np.clip(img, 0, 255)

    # 全局亮度变化
    brightness_shift = random.uniform(-15, 15)
    img = np.clip(img + brightness_shift, 0, 255)

    # 高斯噪声 (模拟相机 sensor noise)
    noise = np.random.randn(size, size) * NOISE_LEVEL * random.uniform(0.5, 1.5)
    img = np.clip(img + noise, 0, 255)

    # 轻微运动模糊 (10% 概率)
    if random.random() < 0.10:
        ksize = random.choice([3, 5])
        kernel = np.zeros((ksize, ksize))
        angle = random.uniform(0, 360)
        # 简化为水平或垂直模糊
        if random.random() < 0.5:
            kernel[ksize // 2, :] = 1.0 / ksize
        else:
            kernel[:, ksize // 2] = 1.0 / ksize
        img = cv2.filter2D(img.astype(np.float32), -1, kernel)

    img = np.clip(img, 0, 255).astype(np.uint8)
    return img


# ═══════════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════════

def generate_dataset(total: int, val_split: float, out_dir: str):
    """生成完整数据集"""
    train_dir = os.path.join(out_dir, "train")
    val_dir   = os.path.join(out_dir, "val")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(val_dir, exist_ok=True)

    n_val = int(total * val_split)
    n_train = total - n_val

    labels: list[tuple[str, int, str]] = []  # (filename, count, split)

    # 分层采样: 确保每个 count 值在训练/验证集中都有足够的样本
    counts_per_class = total // (MAX_COUNT + 1) + 1

    print(f"Generating {total} images (train={n_train}, val={n_val})")
    print(f"Count range: 0 ~ {MAX_COUNT}")
    print(f"Image size: {IMG_SIZE}×{IMG_SIZE}")
    print()

    # 首先生成每个 count 类的图像
    for count in range(0, MAX_COUNT + 1):
        n = counts_per_class
        for i in tqdm(range(n), desc=f"Count={count:2d}", unit="img"):
            img = generate_image(count, IMG_SIZE)
            split = "val" if random.random() < val_split else "train"
            split_dir = train_dir if split == "train" else val_dir
            filename = f"{count:02d}_{i:05d}.jpg"
            filepath = os.path.join(split_dir, filename)
            cv2.imwrite(filepath, img)
            labels.append((filename, count, split))

    # 随机追加更多样本以填充总数
    remaining = total - len(labels)
    if remaining > 0:
        for i in tqdm(range(remaining), desc="Extra samples", unit="img"):
            count = random.randint(0, MAX_COUNT)
            img = generate_image(count, IMG_SIZE)
            split = "val" if random.random() < val_split else "train"
            split_dir = train_dir if split == "train" else val_dir
            filename = f"extra_{i:05d}_{count:02d}.jpg"
            filepath = os.path.join(split_dir, filename)
            cv2.imwrite(filepath, img)
            labels.append((filename, count, split))

    # 保存标签 CSV
    csv_path = os.path.join(out_dir, "labels.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "count", "split"])
        writer.writerows(labels)

    # 统计
    train_labels = [l for l in labels if l[2] == "train"]
    val_labels = [l for l in labels if l[2] == "val"]
    print(f"\nDone! Train: {len(train_labels)}  Val: {len(val_labels)}")
    print(f"Saved to: {os.path.abspath(out_dir)}")
    print(f"Labels:   {csv_path}")


def show_samples(out_dir: str, n: int = 9):
    """显示生成的样例图像"""
    import matplotlib.pyplot as plt

    train_dir = os.path.join(out_dir, "train")
    files = sorted(os.listdir(train_dir))[:n * 2]

    samples = []
    for f in files:
        if f.endswith(".jpg"):
            parts = f.split("_")
            count = int(parts[0]) if parts[0].isdigit() else int(parts[-1].split(".")[0])
            img = cv2.imread(os.path.join(train_dir, f), cv2.IMREAD_GRAYSCALE)
            samples.append((img, count))
        if len(samples) >= n:
            break

    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(10, 3.5 * rows))
    axes = axes.flatten()

    for i, (img, count) in enumerate(samples):
        axes[i].imshow(img, cmap="gray", vmin=0, vmax=255)
        axes[i].set_title(f"Count = {count}", fontsize=12)
        axes[i].axis("off")

    for j in range(len(samples), len(axes)):
        axes[j].axis("off")

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "samples.png"), dpi=100)
    plt.show()
    print(f"Sample image saved to: {os.path.join(out_dir, 'samples.png')}")


# ═══════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════

def main():
    global IMG_SIZE, MAX_COUNT

    # 使用局部变量做默认值，避免 global 声明前的引用
    default_total = TOTAL_SAMPLES
    default_max_count = MAX_COUNT
    default_img_size = IMG_SIZE
    default_out_dir = OUT_DIR

    parser = argparse.ArgumentParser(
        description="生成钢球合成数据集"
    )
    parser.add_argument("--total", type=int, default=default_total,
                        help=f"总样本数 (默认: {default_total})")
    parser.add_argument("--max-count", type=int, default=default_max_count,
                        help=f"最大钢球数量 (默认: {default_max_count})")
    parser.add_argument("--val-split", type=float, default=0.15,
                        help="验证集比例 (默认: 0.15)")
    parser.add_argument("--img-size", type=int, default=default_img_size,
                        help=f"图像尺寸 (默认: {default_img_size})")
    parser.add_argument("--out-dir", type=str, default=default_out_dir,
                        help=f"输出目录 (默认: {default_out_dir})")
    parser.add_argument("--show-samples", action="store_true",
                        help="生成后显示样例图像")
    args = parser.parse_args()

    IMG_SIZE = args.img_size
    MAX_COUNT = args.max_count

    generate_dataset(args.total, args.val_split, args.out_dir)

    if args.show_samples:
        show_samples(args.out_dir)


if __name__ == "__main__":
    main()
