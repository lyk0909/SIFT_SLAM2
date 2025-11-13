#!/bin/bash
echo "Building SIFT Vocabulary Generator..."

cd tools
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo ""
echo "Build complete!"
echo "Executable: tools/generate_sift_vocabulary"
echo ""
echo "Usage:"
echo "  ./tools/generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt"
