# Windows 模型转换: TFLite → kmodel

## 为什么需要这一步

K230D 的 KPU（神经网络加速器）只认 `.kmodel` 格式。训练产出的是 `.tflite`，需要 nncase 工具转换。

**这一步骤在 Windows 上进行。** nncase 在 Fedora 44 上有兼容性 bug，Windows 版本稳定。

---

## 1. 安装 Python 3.12

https://www.python.org/downloads/

安装时勾选 "Add Python to PATH"。

## 2. 安装 Windows Terminal（可选，更好用）

在 Microsoft Store 搜索 "Windows Terminal" 安装。

## 3. 下载 nncase

打开 https://github.com/kendryte/nncase/releases/tag/v2.11.0

下载这两个文件：
- `nncase-2.11.0-cp312-cp312-win_amd64.whl`    （编译器）
- `nncase_kpu-2.11.0-py2.py3-none-win_amd64.whl` （K230 目标支持）

或者用命令行：
```cmd
curl -LO https://github.com/kendryte/nncase/releases/download/v2.11.0/nncase-2.11.0-cp312-cp312-win_amd64.whl
curl -LO https://github.com/kendryte/nncase/releases/download/v2.11.0/nncase_kpu-2.11.0-py2.py3-none-win_amd64.whl
```

## 4. 安装 nncase

```cmd
pip install nncase-2.11.0-cp312-cp312-win_amd64.whl
pip install nncase_kpu-2.11.0-py2.py3-none-win_amd64.whl
```

## 5. 转换模型

把 Linux 上训练好的 `models/steelball_small.tflite` 复制到 Windows。

创建 `convert.py`：

```python
import nncase, os

TFLITE = "steelball_small.tflite"
KMODEL = "model.kmodel"

# 模型参数（需与训练时一致）
IMG_SIZE = 128        # 输入尺寸
CHANNELS = 1          # 灰度图 = 1 通道
NUM_CLASSES = 11      # 输出类别数 (0~10 个球)

print(f"Input:  {TFLITE}")
print(f"Output: {KMODEL}")
print(f"Shape:  (1, {CHANNELS}, {IMG_SIZE}, {IMG_SIZE})  # NCHW")
print()

# 编译选项
opts = nncase.CompileOptions()
opts.target = "k230"
opts.input_type = "float32"
opts.input_shape = (1, CHANNELS, IMG_SIZE, IMG_SIZE)
opts.input_range = [0.0, 1.0]

# 编译
compiler = nncase.Compiler(opts)
io = nncase.ImportOptions()

with open(TFLITE, "rb") as f:
    compiler.import_tflite(f.read(), io)

print("Compiling...")
compiler.compile()

print("Generating kmodel...")
compiler.gencode(KMODEL)

size_kb = os.path.getsize(KMODEL) / 1024
print(f"\nDone! {KMODEL} ({size_kb:.0f} KB)")
```

运行：
```cmd
python convert.py
```

## 6. 部署到 K230D

把生成的 `model.kmodel` 和项目里的 `deploy/ball_count_nn.py` 一起复制到 SD 卡根目录：

```
SD 卡:
├── main.py          ← 就是 ball_count_nn.py
├── model.kmodel     ← 转换出来的神经网络模型
└── ...
```

插入开发板，上电自动运行。脚本检测到 kmodel 自动用 KPU 推理，否则回退 CV 模式。

---

## 常见问题

### Q: import nncase 报错

确保先安装了 .NET Runtime 8.0（nncase 依赖 .NET）：
https://dotnet.microsoft.com/en-us/download/dotnet/8.0

选 ".NET Runtime 8.0.x" → Windows x64。

### Q: 转换报错 "unsupported op"

模型使用了 nncase 不支持的算子。本项目模型只用了 Conv2D / MaxPool / ReLU / Dense / Softmax，全部兼容。
