FROM ubuntu:22.04

# sys
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Beijing

RUN apt update && apt upgrade -y
RUN apt install -y software-properties-common wget curl build-essential

RUN apt install python3-launchpadlib -y
RUN rm -rf ~/.launchpadlib/api.launchpad.net/cache/

RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3 get-pip.py

# cpp
RUN apt install -y cmake gdb gcc g++

# eigen
RUN apt install -y libeigen3-dev

# suitesparse
RUN apt install -y libsuitesparse-dev

# git
RUN apt install -y git

# protobuf
RUN apt install -y protobuf-compiler

# grpc
RUN apt install -y autoconf automake libtool pkg-config
RUN git clone --recurse-submodules -b v1.66.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc /var/local/git/grpc
RUN cd /var/local/git/grpc && \
    mkdir -p cmake/build && \
    cd cmake/build && \
    cmake -DgRPC_INSTALL=ON \
    -DgRPC_BUILD_TESTS=OFF \
    -DCMAKE_CXX_STANDARD=17 \
    ../.. && \
    make -j 8 && \
    make install
