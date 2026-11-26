# Ubuntu 22.04 版本

两个架构目录：

```text
x64/    x86_64 (amd64)
arm/    arm64 (aarch64)
```

同时提供打包好的源码包：

```text
调试助手-ubuntu22.04-x64.tar.gz
调试助手-ubuntu22.04-arm64.tar.gz
```

每个目录都包含：

- `../调试助手.py`：通用源码（唯一一份，各平台目录只放脚本）；
- `run.sh`：直接运行；
- `build.sh`：用 PyInstaller 打成单文件；
- `requirements.txt`；
- `README.md`：该架构的依赖/权限/构建说明。

## 为什么没有直接放 ELF

PyInstaller 不能跨架构打包。Linux 二进制必须在对应架构的 Ubuntu 22.04 上构建：

```bash
# x64 机器
cd x64 && ./build.sh

# arm64 机器
cd arm && ./build.sh
```

源码本身是纯 Python，两个架构完全一致，可以先用 `run.sh` 直接跑。

## 依赖

```bash
sudo apt update
sudo apt install -y python3 python3-tk python3-pip
python3 -m pip install --user -r requirements.txt
sudo usermod -aG dialout $USER
# 重新登录
```
