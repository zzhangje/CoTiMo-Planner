#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$ROOT_DIR/build-linux"
VCPKG_ROOT="$ROOT_DIR/vcpkg"

mkdir -p "$BUILD_DIR"
mkdir -p "$VCPKG_ROOT"

if [ ! -f "$VCPKG_ROOT/vcpkg" ]; then
    echo "正在安装 vcpkg..."
    git clone https://github.com/Microsoft/vcpkg.git "$VCPKG_ROOT"
    pushd "$VCPKG_ROOT"
    ./bootstrap-vcpkg.sh
    popd
fi

export VCPKG_ROOT="$VCPKG_ROOT"

pushd "$BUILD_DIR"
cmake -B . -S "$ROOT_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
    -DVCPKG_TARGET_TRIPLET=x64-linux \
    -DCMAKE_BUILD_TYPE=Release

cmake --build . --config Release
popd

echo "构建完成！"
echo "构建目录: $BUILD_DIR"
echo "Vcpkg 目录: $VCPKG_ROOT" 