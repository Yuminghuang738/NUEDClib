# K230D 钢球数量识别

基于神经网络的钢球数量识别项目，运行在**立创·庐山派 Lite-K230D-CanMV** 开发板上。

PC 端负责模型训练，K230D 端负责推理部署。

## 项目结构

```
K230D/
├── README.md                     # 项目说明（本文件）
├── requirements.txt              # Python 训练环境依赖
│
├── dataset/
│   └── generate_synthetic.py     # 合成钢球数据集生成
│
├── training/
│   ├── model.py                  # CNN 模型架构 (Tiny/Small/Medium)
│   ├── train.py                  # 训练脚本
│   └── evaluate.py               # 模型评估
│
├── models/
│   ├── steelball_small.h5        # 训练好的 Keras 模型
│   └── steelball_small.tflite    # TFLite 模型（拿去 Windows 转 kmodel）
│
├── deploy/
│   └── ball_count_nn.py          # K230D 部署脚本（复制到 SD 卡）
│
└── tools/
    └── convert_windows.md        # Windows 上 TFLite→kmodel 转换指南
```

## 技术方案

### 模型: CNN 分类器

将钢球计数转化为分类问题：输入图像，输出 0~10（共 11 类，对应 0~10 个钢球）。

```
输入: 128×128 灰度图
  → Conv3×3×16 + Conv3×3×16 + MaxPool   (128→64)
  → Conv3×3×32 + Conv3×3×32 + MaxPool   (64→32)
  → Conv3×3×64 + Conv3×3×64 + MaxPool   (32→16)
  → Conv3×3×128 + Conv3×3×128 + MaxPool (16→8)
  → Conv3×3×128 + MaxPool                (8→4)
  → Flatten → Dense(64) → Dense(11) softmax
输出: 0~10 (预测钢球数量)
```

仅使用 K230D KPU 原生支持的算子：Conv2D、MaxPool、ReLU、Dense、Softmax。

### 当前模型性能

| 指标 | 值 | 说明 |
|------|-----|------|
| 准确率 | 71.5% | 精确匹配 |
| 平均误差 | 0.45 个 | 平均差不到半个球 |
| ±1 容差 | 89.2% | 差一个球以内 |
| 模型大小 | 2.2 MB | TFLite float32 |
| 训练样本 | 3000 合成图 | 可扩展到数万 |
| 图像尺寸 | 128×128 | 灰度 |

### 小数量（0~5 个球）时准确率 89%~100%，适合大多数实际场景。

## 快速开始

### 环境配置（Linux，仅需一次）

```bash
cd K230D
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 生成训练数据

```bash
source .venv/bin/activate
python dataset/generate_synthetic.py --total 10000 --max-count 10 --img-size 128
```

参数说明:
- `--total` 总样本数（默认 3000，越多越准）
- `--max-count` 最大钢球数（默认 10 → 输出 0~10 共 11 类）
- `--img-size` 图像尺寸（默认 128）

### 训练模型

```bash
source .venv/bin/activate
python training/train.py --model small --epochs 50 --batch 32 --img-size 128
```

三种模型可选:

| 模型 | 参数量 | 速度 | 适用 |
|------|--------|------|------|
| tiny | ~35K | 最快 | 追求帧率 |
| small | ~200K | 中等 | **推荐** |
| medium | ~500K | 较慢 | 追求精度 |

输出:
- `models/steelball_small.h5` — Keras 模型
- `models/steelball_small.tflite` — 用于转换 kmodel

### 评估模型

```bash
source .venv/bin/activate
python training/evaluate.py --model models/steelball_small.h5 --img-size 128 --dataset-dir dataset
```

### 转换 kmodel → 部署到 K230D

1. 把 `models/steelball_small.tflite` 复制到 Windows
2. 按照 `tools/convert_windows.md` 在 Windows 上转为 `model.kmodel`
3. 把 `model.kmodel` 和 `deploy/ball_count_nn.py` 复制到 SD 卡:

```
SD 卡:
├── main.py          ← ball_count_nn.py 重命名
└── model.kmodel     ← 转换出来的模型
```

4. 插入 K230D，上电运行

### 部署脚本工作模式

| SD 卡有 model.kmodel? | 推理方式 | 特点 |
|----------------------|---------|------|
| ✅ 有 | KPU 神经网络 | 高精度（71.5%），抗干扰 |
| ❌ 没有 | cv_lite blob 检测 | 传统 CV，开箱即用 |

## 提高精度的方法

1. **增加样本量**: `--total 50000`
2. **增大图像**: `--img-size 224`（同时需要修 改 model.py 的 IMG_SIZE）
3. **更多轮数**: `--epochs 100`
4. **用更大模型**: `--model medium`
5. **真实数据微调**: 拍真实钢球照片，按 `{count}_{id}.jpg` 命名放入 `dataset/real/` 目录，用 `--dataset-dir dataset/real` 微调

## 与传统 CV 对比

| 维度 | 传统 CV | 神经网络 |
|------|---------|---------|
| 抗噪 | 弱（依赖阈值） | 强 |
| 光照适应 | 差 | 好 |
| 遮挡处理 | 差 | 较好 |
| 速度 | 快 | 中等 |
| 泛化 | 不适用其他物体 | 换数据即可 |
