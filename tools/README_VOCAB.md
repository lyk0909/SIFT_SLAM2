# SIFT 词汇表生成工具使用说明

## 编译工具

```bash
./build_vocabulary_tool.sh
```

## 使用方法

### 基本用法

```bash
./tools/generate_sift_vocabulary <图像目录> <输出词汇表文件>
```

### 完整示例

```bash
# 从 EuRoC MH_01_easy 数据集生成 SIFT 词汇表
./tools/generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt
```

### 高级选项

```bash
./tools/generate_sift_vocabulary <图像目录> <输出文件> [选项]

选项:
  -k <int>     分支因子 (默认: 10)
  -L <int>     树的深度 (默认: 6)
  -f <int>     每张图像的最大特征数 (默认: 2000)
```

### 示例：自定义参数

```bash
# 使用更大的词汇表 (k=10, L=6 会生成约 10^6 = 1,000,000 个词)
./tools/generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt -k 10 -L 6 -f 2000

# 使用较小的词汇表进行快速测试 (k=10, L=4 会生成约 10,000 个词)
./tools/generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc_small.txt -k 10 -L 4 -f 1000
```

## 注意事项

1. **处理时间**: 生成完整词汇表可能需要 **几小时到十几小时**，取决于：
   - 图像数量
   - 每张图像的特征数
   - 词汇表大小 (k 和 L 参数)
   - 计算机性能

2. **内存需求**: 
   - 小词汇表 (L=4): 约 2-4 GB
   - 中等词汇表 (L=5): 约 4-8 GB
   - 大词汇表 (L=6): 约 8-16 GB

3. **建议的参数**:
   ```
   快速测试:  k=10, L=4, f=1000  (约 10,000 词, 10-30 分钟)
   中等质量:  k=10, L=5, f=1500  (约 100,000 词, 1-3 小时)
   高质量:    k=10, L=6, f=2000  (约 1,000,000 词, 3-10 小时)
   ```

## 使用生成的词汇表

生成词汇表后，需要在 ORB-SLAM2 中使用它：

```bash
# 运行单目 SLAM，使用新的 SIFT 词汇表
./Examples/Monocular/mono_tum Vocabulary/SIFTvoc.txt Examples/Monocular/TUM1.yaml PATH_TO_SEQUENCE
```

## 故障排除

### 错误: "No images found"
- 检查图像目录路径是否正确
- 确保目录中有 .png 文件

### 错误: "No features extracted"
- 检查图像是否损坏
- 尝试降低特征数 (-f 参数)

### 内存不足
- 减小词汇表大小 (降低 L 参数)
- 减少每张图像的特征数 (-f 参数)
- 使用更少的图像

### 进程被杀死 (Killed)
- 系统内存不足，请增加交换空间或降低参数

## 快速测试脚本

创建小规模测试：

```bash
# 快速测试 (约 5-10 分钟)
./tools/generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc_test.txt -k 9 -L 3 -f 500
```
