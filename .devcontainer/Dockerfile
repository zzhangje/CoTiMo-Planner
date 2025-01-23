FROM ubuntu:22.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Beijing
ENV VCPKG_ROOT=/vcpkg
ENV VCPKG_DOWNLOADS_ROOT=/vcpkg/downloads
ENV VCPKG_DEFAULT_BINARY_CACHE=/vcpkg/binary_cache
# 配置 vcpkg 使用华为源
ENV VCPKG_FORCE_SYSTEM_BINARIES=1
ENV X_VCPKG_ASSET_SOURCES=x-azurl,http://mirrors.huaweicloud.com/vcpkg/downloads/,x-azurl,https://github.com/microsoft/vcpkg/raw/master/scripts/

# 配置华为源
RUN sed -i 's@//.*archive.ubuntu.com@//repo.huaweicloud.com@g' /etc/apt/sources.list && \
    sed -i 's@//.*security.ubuntu.com@//repo.huaweicloud.com@g' /etc/apt/sources.list

# 安装系统依赖和开发工具
RUN apt update && apt upgrade -y && \
    apt install -y \
    software-properties-common \
    wget \
    curl \
    build-essential \
    cmake \
    gdb \
    gcc \
    g++ \
    git \
    ninja-build \
    pkg-config \
    zip \
    unzip \
    tar \
    python3 \
    python3-pip \
    python3-launchpadlib \
    libsuitesparse-dev \
    ccache \
    ninja-build \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf ~/.launchpadlib/api.launchpad.net/cache/

# 配置 ccache
ENV CCACHE_DIR=/ccache
ENV PATH="/usr/lib/ccache:${PATH}"
RUN mkdir -p ${CCACHE_DIR}

# 配置 pip 华为源
RUN pip config set global.index-url https://mirrors.huaweicloud.com/repository/pypi/simple && \
    pip config set global.trusted-host mirrors.huaweicloud.com

# 安装 vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git ${VCPKG_ROOT} && \
    ${VCPKG_ROOT}/bootstrap-vcpkg.sh && \
    ln -s ${VCPKG_ROOT}/vcpkg /usr/local/bin/vcpkg && \
    mkdir -p ${VCPKG_DOWNLOADS_ROOT} ${VCPKG_DEFAULT_BINARY_CACHE}

# 设置工作目录
WORKDIR /workspaces/Cyber-Planner-2025

# 安装 vcpkg 依赖
RUN vcpkg install --clean-after-build

# 预编译步骤
RUN mkdir -p build && \
    cd build && \
    cmake -G "Unix Makefiles" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_TOOLCHAIN_FILE=/vcpkg/scripts/buildsystems/vcpkg.cmake \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        .. || true && \
    make -j$(nproc) || true && \
    cd ..

# 清理
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
