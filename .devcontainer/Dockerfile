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

# suitesparse
RUN apt install -y libsuitesparse-dev

# git
RUN apt install -y git

# vcpkg dependencies
RUN apt install -y zip unzip tar pkg-config

# vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git
RUN cd vcpkg && ./bootstrap-vcpkg.sh
RUN cd vcpkg && ./vcpkg install eigen3 grpc protobuf
