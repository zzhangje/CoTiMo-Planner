#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$ROOT_DIR/build-linux"

echo "正在清理构建目录..."

if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
    echo "已删除构建目录: $BUILD_DIR"
fi

echo "清理完成！" 