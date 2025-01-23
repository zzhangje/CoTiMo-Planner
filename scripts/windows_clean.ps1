#!/usr/bin/env pwsh

# 设置路径变量
$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$ROOT_DIR = Split-Path -Parent $SCRIPT_DIR
$BUILD_DIR = Join-Path $ROOT_DIR "build-windows"

# 清理构建目录
if (Test-Path $BUILD_DIR) {
    Write-Host "正在清理构建目录: $BUILD_DIR"
    Remove-Item -Recurse -Force $BUILD_DIR
    Write-Host "清理完成！"
} else {
    Write-Host "构建目录不存在，无需清理"
} 