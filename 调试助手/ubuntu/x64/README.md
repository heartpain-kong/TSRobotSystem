# 调试助手 — Ubuntu 22.04 x86_64

## 依赖

```bash
sudo apt update
sudo apt install -y python3 python3-tk python3-pip
python3 -m pip install --user -r requirements.txt
sudo usermod -aG dialout $USER
# 重新登录，串口权限才生效
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

生成 `dist/调试助手`（x86_64 ELF）。PyInstaller **不能跨架构**，ARM 包必须在
ARM64 Ubuntu 22.04 上构建。

## 串口

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

CH340 一般是 `/dev/ttyUSB0`。TX/RX 交叉接，GND 共地。
