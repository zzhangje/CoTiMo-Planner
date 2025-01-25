#!/usr/bin/env pwsh

$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$ROOT_DIR = Split-Path -Parent $SCRIPT_DIR
$BUILD_DIR = Join-Path $ROOT_DIR "build-windows"

if (Test-Path $BUILD_DIR) {
    Write-Host "Cleaning build directory: $BUILD_DIR"
    Remove-Item -Recurse -Force $BUILD_DIR
    Write-Host "Cleaning finished!"
} else {
    Write-Host "Build directory not found: $BUILD_DIR"
    Write-Host "Nothing to clean."
} 