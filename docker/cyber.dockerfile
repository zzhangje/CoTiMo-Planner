FROM ubuntu:20.04

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Beijing

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    libgtk2.0-dev \
    lsb-release \
    net-tools \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    git \
    wget \
    curl \
    htop \
    xterm \
    libpcap-dev \
    binutils-dev \
    libdw-dev \
    libdwarf-dev \
    gdb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
