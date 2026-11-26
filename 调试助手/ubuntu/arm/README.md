# 调试助手 — Ubuntu 22.04 ARM64

适用于树莓派 4/5、Jetson、RK3588 等 arm64 Ubuntu 22.04。

## 依赖

```bash
sudo apt update
sudo apt install -y python3 python3-tk python3-pip
python3 -m pip install --user -r requirements.txt
sudo usermod -aG dialout $USER
# 重新登录
```

## 运行

```bash
chmod +x run.sh
./run.sh
```

## 打包

```bash
chmod +x build.sh
./build.sh
```

生成 `dist/调试助手`（aarch64 ELF）。

PyInstaller 不能跨架构：x64 包在 x86_64 构建，ARM 包在 arm64 构建。
