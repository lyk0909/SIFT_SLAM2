#!/bin/bash

echo "=========================================="
echo "SIFT Vocabulary Generator - Ultra Small Test"
echo "=========================================="
echo ""
echo "This script will generate a TINY SIFT vocabulary for testing."
echo "Parameters: k=6, L=2, maxFeatures=200, maxImages=200"
echo "Expected time: 1-2 minutes"
echo ""

# 检查工具是否存在
if [ ! -f "tools/generate_sift_vocabulary" ]; then
    echo "Error: generate_sift_vocabulary not found."
    echo "Please run ./build_vocabulary_tool.sh first"
    exit 1
fi

# 检查图像目录是否存在
if [ ! -d "data/MH_01_easy/mav0/cam0/data" ]; then
    echo "Error: Image directory not found: data/MH_01_easy/mav0/cam0/data"
    exit 1
fi

# 创建临时目录，只复制部分图像
TEMP_DIR="/tmp/sift_vocab_test_images"
rm -rf $TEMP_DIR
mkdir -p $TEMP_DIR

echo "Copying first 200 images to temporary directory..."
ls data/MH_01_easy/mav0/cam0/data/*.png | head -200 | while read img; do
    cp "$img" "$TEMP_DIR/"
done

echo "Found $(ls $TEMP_DIR/*.png | wc -l) images"
echo ""

# 创建 Vocabulary 目录
mkdir -p Vocabulary

echo "Starting vocabulary generation..."
echo "Output file: Vocabulary/SIFTvoc_tiny.txt"
echo ""

# 运行生成工具
./tools/generate_sift_vocabulary \
    $TEMP_DIR \
    Vocabulary/SIFTvoc_tiny.txt \
    -k 6 \
    -L 2 \
    -f 200

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Vocabulary generation completed!"
    echo "=========================================="
    echo ""
    echo "File: Vocabulary/SIFTvoc_tiny.txt"
    ls -lh Vocabulary/SIFTvoc_tiny.txt 2>/dev/null
    echo ""
    rm -rf $TEMP_DIR
else
    echo ""
    echo "Error: Vocabulary generation failed."
    rm -rf $TEMP_DIR
    exit 1
fi
