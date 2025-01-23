#!/usr/bin/env pwsh

$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$ROOT_DIR = Split-Path -Parent $SCRIPT_DIR
$BUILD_DIR = Join-Path $ROOT_DIR "build-windows"
$VCPKG_ROOT = Join-Path $ROOT_DIR "vcpkg"

New-Item -ItemType Directory -Force -Path $BUILD_DIR | Out-Null
New-Item -ItemType Directory -Force -Path $VCPKG_ROOT | Out-Null

if (-not (Test-Path (Join-Path $VCPKG_ROOT "vcpkg.exe"))) {
    Write-Host "正在安装 vcpkg..."
    git clone https://github.com/Microsoft/vcpkg.git $VCPKG_ROOT
    Push-Location $VCPKG_ROOT
    .\bootstrap-vcpkg.bat
    Pop-Location
}

$env:VCPKG_ROOT = $VCPKG_ROOT

Push-Location $BUILD_DIR
cmake -B . -S $ROOT_DIR `
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
    -DVCPKG_TARGET_TRIPLET=x64-windows `
    -DCMAKE_BUILD_TYPE=Release

cmake --build . --config Release
Pop-Location

Write-Host "构建完成！"
Write-Host "构建目录: $BUILD_DIR"
Write-Host "Vcpkg 目录: $VCPKG_ROOT" 