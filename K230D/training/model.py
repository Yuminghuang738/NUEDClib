"""
钢球数量识别 —— 模型架构定义。

提供多个可选模型:
  - SteelBallCNN_Tiny:   极轻量, ~15K 参数, 适合 K230D KPU 实时推理
  - SteelBallCNN_Small:  轻量,   ~80K 参数, 精度更好
  - SteelBallCNN_Medium: 中等,  ~300K 参数, 高精度

K230D KPU 兼容性说明:
  KPU 对运算有限制，本模型只使用 KPU 原生支持的算子:
    - Conv2D (3×3, 1×1)
    - MaxPool2D
    - ReLU
    - FullyConnected (Dense)
    - Softmax
  不使用 BN (用 pre-activation 或跳过)、DepthwiseConv、SE 等算子。

输入: 224×224 灰度图 (单通道)
输出: (MAX_COUNT+1) 类 softmax, 对应 0~MAX_COUNT 个钢球
"""

import tensorflow as tf


# ── 常量 ──
MAX_COUNT = 15                    # 最大钢球数 → 输出类别数 = MAX_COUNT + 1
IMG_SIZE  = 224                   # 输入图像尺寸
CHANNELS  = 1                     # 灰度图


# ═══════════════════════════════════════════════════════════════════
# 模型定义
# ═══════════════════════════════════════════════════════════════════

def SteelBallCNN_Tiny(num_classes: int = MAX_COUNT + 1,
                      input_shape: tuple = (IMG_SIZE, IMG_SIZE, CHANNELS)):
    """极轻量 CNN — 适合 K230D KPU 实时推理 (30+ FPS)。

    Architecture:
        Conv3×3×8  → MaxPool → 112×112×8
        Conv3×3×16 → MaxPool → 56×56×16
        Conv3×3×32 → MaxPool → 28×28×32
        Conv3×3×64 → MaxPool → 14×14×64
        Conv3×3×64 → MaxPool → 7×7×64
        Flatten → Dense(48) → Dense(num_classes)

    Params: ~35K
    """
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),

        # Block 1: 224 → 112
        tf.keras.layers.Conv2D(8, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(8, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 2: 112 → 56
        tf.keras.layers.Conv2D(16, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(16, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 3: 56 → 28
        tf.keras.layers.Conv2D(32, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 4: 28 → 14
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 5: 14 → 7
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Head
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(48, activation="relu"),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(num_classes, activation="softmax"),
    ], name="SteelBallCNN_Tiny")
    return model


def SteelBallCNN_Small(num_classes: int = MAX_COUNT + 1,
                       input_shape: tuple = (IMG_SIZE, IMG_SIZE, CHANNELS)):
    """轻量 CNN — 精度与速度的平衡点。

    Architecture:
        Conv3×3×16 → Conv3×3×16 → MaxPool → 112×112×16
        Conv3×3×32 → Conv3×3×32 → MaxPool → 56×56×32
        Conv3×3×64 → Conv3×3×64 → MaxPool → 28×28×64
        Conv3×3×128 → Conv3×3×128 → MaxPool → 14×14×128
        Conv3×3×128 → MaxPool → 7×7×128
        Flatten → Dense(64) → Dropout → Dense(num_classes)

    Params: ~200K
    """
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),

        # Block 1: 224 → 112
        tf.keras.layers.Conv2D(16, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(16, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 2: 112 → 56
        tf.keras.layers.Conv2D(32, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(32, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 3: 56 → 28
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 4: 28 → 14
        tf.keras.layers.Conv2D(128, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(128, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 5: 14 → 7
        tf.keras.layers.Conv2D(128, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Head
        tf.keras.layers.Flatten(),                        # 7×7×128 = 6272
        tf.keras.layers.Dense(64, activation="relu"),
        tf.keras.layers.Dropout(0.4),
        tf.keras.layers.Dense(num_classes, activation="softmax"),
    ], name="SteelBallCNN_Small")
    return model


def SteelBallCNN_Medium(num_classes: int = MAX_COUNT + 1,
                        input_shape: tuple = (IMG_SIZE, IMG_SIZE, CHANNELS)):
    """中等规模 CNN — 追求最高精度。

    Architecture: 类似 VGG 风格的堆叠卷积
    Params: ~500K
    """
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),

        # Block 1: 224 → 112
        tf.keras.layers.Conv2D(32, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(32, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 2: 112 → 56
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(64, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 3: 56 → 28
        tf.keras.layers.Conv2D(128, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(128, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 4: 28 → 14
        tf.keras.layers.Conv2D(256, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.Conv2D(256, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Block 5: 14 → 7
        tf.keras.layers.Conv2D(256, 3, strides=1, padding="same", activation="relu"),
        tf.keras.layers.MaxPooling2D(2),

        # Head
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(128, activation="relu"),
        tf.keras.layers.Dropout(0.5),
        tf.keras.layers.Dense(64, activation="relu"),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(num_classes, activation="softmax"),
    ], name="SteelBallCNN_Medium")
    return model


# ═══════════════════════════════════════════════════════════════════
# 工厂函数
# ═══════════════════════════════════════════════════════════════════

_MODELS = {
    "tiny":   SteelBallCNN_Tiny,
    "small":  SteelBallCNN_Small,
    "medium": SteelBallCNN_Medium,
}


def create_model(name: str = "small", num_classes: int = MAX_COUNT + 1,
                 input_shape: tuple = (IMG_SIZE, IMG_SIZE, CHANNELS)):
    """创建模型的便捷函数。

    Args:
        name: "tiny" | "small" | "medium"
        num_classes: 输出类别数 (默认 16, 对应 0~15 个球)
        input_shape: 输入张量形状

    Returns:
        tf.keras.Model
    """
    name = name.lower()
    if name not in _MODELS:
        raise ValueError(f"Unknown model '{name}'. Choose from: {list(_MODELS.keys())}")
    return _MODELS[name](num_classes=num_classes, input_shape=input_shape)


# ═══════════════════════════════════════════════════════════════════
# 自测
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    for name in ["tiny", "small", "medium"]:
        model = create_model(name)
        params = model.count_params()
        print(f"{name:>8}: {params:>8,} params")
        model.summary()
        print()
