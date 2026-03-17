# 使用更稳定的 Ubuntu 20.04 基础镜像
FROM ubuntu:20.04

# 设置环境变量，防止 apt-get 在安装时弹出交互式配置界面（如地理位置选择）
ENV DEBIAN_FRONTEND=noninteractive

# 1. 更新源并安装所有必要的构建工具
RUN set -x && \
    apt-get update && \
    apt-get install -y \
        build-essential \
        git \
        time \
        openocd \
        tup \
        # Ubuntu 20.04 默认自带 python3 (3.8版本)，足以胜任 ODrive 编译
        python3 \
        python3-pip \
        python3-yaml \
        python3-jinja2 \
        python3-jsonschema \
        # 交叉编译链：用于 ARM 芯片
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
    && \
    # 2. 建立 Python 软链接，确保执行 python 时使用的是 python3
    ln -sf /usr/bin/python3 /usr/bin/python && \
    # 3. 清理缓存以减小镜像体积
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    # 4. 创建工作目录
    mkdir -p /ODrive

# 设置工作目录
WORKDIR /ODrive/Firmware

# 注意：运行容器时需将宿主机的 ODrive 源码挂载到容器内的 /ODrive
# 例如：docker run -v /your/path/to/ODrive:/ODrive <image_name>

CMD \
    # 重新生成版本信息和接口代码
    mkdir -p autogen && \
    python ../tools/odrive/version.py --output autogen/version.c && \
    \
    python interface_generator_stub.py \
    --definitions odrive-interface.yaml \
    --template ../tools/enums_template.j2 \
    --output ../tools/odrive/enums.py && \
    \
    python interface_generator_stub.py \
    --definitions odrive-interface.yaml \
    --template ../tools/arduino_enums_template.j2 \
    --output ../Arduino/ODriveArduino/ODriveEnums.h && \
    \
    # Tup 在 Docker 中运行的特殊处理
    # 提示：由于容器默认禁用了 FUSE，我们需要初始化并生成独立的 build.sh 脚本运行
    tup init && \
    tup generate build.sh && \
    ./build.sh