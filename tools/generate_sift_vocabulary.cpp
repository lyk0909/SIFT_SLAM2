/**
 * 从图像中提取 SIFT 特征并生成 DBoW2 词汇表
 * 用法: ./generate_sift_vocabulary <image_directory> <output_vocabulary_file>
 * 示例: ./generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt
 */

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// DBoW2
#include "TemplatedVocabulary.h"
#include "FORB.h"

// 自定义 SIFT 特征的 DBoW2 类型
// SIFT 描述符是 128 维浮点向量，我们使用 FORB (256位) 的模板作为基础
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

using namespace std;
using namespace cv;

// 加载图像路径
void loadImages(const string &strImagePath, vector<string> &vstrImageFilenames)
{
    cout << "Loading images from: " << strImagePath << endl;
    
    // 读取目录下所有 .png 文件
    ifstream fAssociation;
    
    // 直接从文件系统读取
    string command = "find " + strImagePath + " -name '*.png' | sort > /tmp/image_list.txt";
    system(command.c_str());
    
    ifstream fImages("/tmp/image_list.txt");
    if(!fImages.is_open())
    {
        cerr << "Error: Cannot open image list file" << endl;
        return;
    }
    
    while(!fImages.eof())
    {
        string s;
        getline(fImages, s);
        if(!s.empty())
        {
            vstrImageFilenames.push_back(s);
        }
    }
    fImages.close();
    
    cout << "Found " << vstrImageFilenames.size() << " images" << endl;
}

// 将 SIFT 浮点描述符转换为二进制描述符（兼容 ORB 格式）
Mat convertSIFTtoORBFormat(const Mat &siftDesc)
{
    // SIFT: 128 float32 -> ORB: 256 bits (32 bytes)
    // 我们通过阈值化将浮点值转换为二进制
    Mat orbDesc(siftDesc.rows, 32, CV_8U);
    
    for(int i = 0; i < siftDesc.rows; i++)
    {
        const float* sift_row = siftDesc.ptr<float>(i);
        uchar* orb_row = orbDesc.ptr<uchar>(i);
        
        // 将 128 个 float 值转换为 256 bits
        // 每个 float 产生 2 bits（通过与均值比较）
        for(int j = 0; j < 32; j++)
        {
            uchar byte_val = 0;
            for(int bit = 0; bit < 8; bit++)
            {
                int idx = j * 4 + bit / 2;
                if(idx < 128)
                {
                    // 使用简单阈值：大于均值的位设为 1
                    float val = sift_row[idx];
                    if(bit % 2 == 0)
                    {
                        byte_val |= (val > 0.1f) ? (1 << bit) : 0;
                    }
                    else
                    {
                        byte_val |= (val > 0.05f) ? (1 << bit) : 0;
                    }
                }
            }
            orb_row[j] = byte_val;
        }
    }
    
    return orbDesc;
}

// 从图像中提取 SIFT 特征
void extractSIFTFeatures(const vector<string> &vstrImageFilenames, 
                          vector<Mat> &features,
                          int maxFeatures = 2000)
{
    cout << "\n=== Extracting SIFT features ===" << endl;
    
    // 创建 SIFT 检测器
    Ptr<SIFT> sift = SIFT::create(maxFeatures);
    
    int nImages = vstrImageFilenames.size();
    features.reserve(nImages);
    
    cout << "Processing " << nImages << " images..." << endl;
    
    for(int i = 0; i < nImages; i++)
    {
        // 读取图像
        Mat image = imread(vstrImageFilenames[i], IMREAD_GRAYSCALE);
        
        if(image.empty())
        {
            cerr << "Error: Cannot read image " << vstrImageFilenames[i] << endl;
            continue;
        }
        
        // 检测并计算 SIFT 特征
        vector<KeyPoint> keypoints;
        Mat descriptors;
        
        sift->detectAndCompute(image, Mat(), keypoints, descriptors);
        
        if(descriptors.empty())
        {
            cerr << "Warning: No features in image " << vstrImageFilenames[i] << endl;
            continue;
        }
        
        // 转换 SIFT 描述符为 ORB 兼容格式（二进制）
        Mat orbDesc = convertSIFTtoORBFormat(descriptors);
        
        features.push_back(orbDesc);
        
        // 显示进度
        if((i + 1) % 50 == 0 || i == nImages - 1)
        {
            cout << "Processed " << (i + 1) << "/" << nImages 
                 << " images, features: " << orbDesc.rows << endl;
        }
    }
    
    cout << "Feature extraction completed!" << endl;
    cout << "Total feature sets: " << features.size() << endl;
}

// 创建词汇表
void createVocabulary(const vector<Mat> &features, 
                      const string &outputFile,
                      int k = 10,      // 分支因子
                      int L = 6)       // 树的深度
{
    cout << "\n=== Creating vocabulary ===" << endl;
    cout << "Parameters:" << endl;
    cout << "  k (branching factor): " << k << endl;
    cout << "  L (depth levels): " << L << endl;
    
    cout << "Converting features for DBoW2..." << endl;
    vector<vector<cv::Mat>> features_vec;
    features_vec.reserve(features.size());
    
    int totalFeatures = 0;
    for(size_t i = 0; i < features.size(); i++)
    {
        vector<cv::Mat> feature_vec;
        feature_vec.reserve(features[i].rows);
        
        for(int j = 0; j < features[i].rows; j++)
        {
            // 每个描述符必须是单行 CV_8U Mat
            cv::Mat desc = features[i].row(j).clone();
            // 确保是连续的
            if(!desc.isContinuous())
            {
                desc = desc.clone();
            }
            // 确保维度正确: 1x32 CV_8U
            assert(desc.rows == 1 && desc.cols == 32 && desc.type() == CV_8U);
            feature_vec.push_back(desc);
        }
        features_vec.push_back(feature_vec);
        totalFeatures += feature_vec.size();
        
        if((i + 1) % 100 == 0)
        {
            cout << "Converted " << (i + 1) << "/" << features.size() << " images" << endl;
        }
    }
    
    cout << "Total features: " << totalFeatures << endl;
    
    // 如果特征太多，进行采样以避免内存问题
    int maxTotalFeatures = 500000;  // 最多使用 50 万个特征
    if(totalFeatures > maxTotalFeatures)
    {
        cout << "Too many features (" << totalFeatures << "), sampling to " << maxTotalFeatures << endl;
        
        // 计算采样率
        float sampleRate = (float)maxTotalFeatures / totalFeatures;
        
        vector<vector<cv::Mat>> sampled_features;
        int sampledCount = 0;
        
        for(size_t i = 0; i < features_vec.size(); i++)
        {
            vector<cv::Mat> sampled_vec;
            for(size_t j = 0; j < features_vec[i].size(); j++)
            {
                if((float)rand() / RAND_MAX < sampleRate)
                {
                    sampled_vec.push_back(features_vec[i][j]);
                    sampledCount++;
                }
            }
            if(!sampled_vec.empty())
            {
                sampled_features.push_back(sampled_vec);
            }
        }
        
        features_vec = sampled_features;
        cout << "Sampled features: " << sampledCount << endl;
    }
    
    cout << "Creating vocabulary. This may take several minutes..." << endl;
    
    // 创建词汇表（使用 ORB 格式，但实际是转换后的 SIFT）
    cout << "Initializing vocabulary structure (k=" << k << ", L=" << L << ")..." << endl;
    
    // 在创建之前验证数据
    cout << "Validating feature data..." << endl;
    cout << "Number of images: " << features_vec.size() << endl;
    
    int validImages = 0;
    int totalValidFeatures = 0;
    for(size_t i = 0; i < features_vec.size(); i++)
    {
        if(features_vec[i].size() > 0)
        {
            validImages++;
            totalValidFeatures += features_vec[i].size();
            
            // 验证第一个特征的格式
            if(i == 0 && features_vec[i].size() > 0)
            {
                const cv::Mat& firstDesc = features_vec[i][0];
                cout << "First descriptor - rows: " << firstDesc.rows 
                     << ", cols: " << firstDesc.cols 
                     << ", type: " << firstDesc.type() 
                     << " (CV_8U=" << CV_8U << ")"
                     << ", continuous: " << firstDesc.isContinuous() << endl;
            }
        }
    }
    
    cout << "Valid images with features: " << validImages << endl;
    cout << "Total valid features: " << totalValidFeatures << endl;
    
    if(validImages == 0 || totalValidFeatures == 0)
    {
        cerr << "Error: No valid features to create vocabulary!" << endl;
        return;
    }
    
    cout << "\nCreating ORBVocabulary object..." << endl;
    ORBVocabulary voc(k, L, DBoW2::TF_IDF, DBoW2::L1_NORM);
    cout << "ORBVocabulary object created successfully" << endl;
    
    // 训练词汇表
    cout << "\n>>> Starting vocabulary training (this may crash if there's a memory issue)..." << endl;
    cout << ">>> If program crashes here, try reducing features or using smaller k/L values" << endl;
    
    voc.create(features_vec);
    
    cout << "\n>>> Vocabulary training completed successfully!" << endl;
    
    cout << "Vocabulary created with " << voc.size() << " words" << endl;
    
    // 保存词汇表
    cout << "Saving vocabulary to " << outputFile << "..." << endl;
    voc.save(outputFile);
    
    cout << "Vocabulary saved successfully!" << endl;
}

void printUsage()
{
    cout << "\nUsage: ./generate_sift_vocabulary <image_directory> <output_vocabulary_file> [options]\n" << endl;
    cout << "Arguments:" << endl;
    cout << "  <image_directory>        Directory containing .png images" << endl;
    cout << "  <output_vocabulary_file> Output vocabulary file path" << endl;
    cout << "\nOptions:" << endl;
    cout << "  -k <int>     Branching factor (default: 10)" << endl;
    cout << "  -L <int>     Depth levels (default: 6)" << endl;
    cout << "  -f <int>     Max features per image (default: 2000)" << endl;
    cout << "\nExample:" << endl;
    cout << "  ./generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt" << endl;
    cout << "  ./generate_sift_vocabulary data/MH_01_easy/mav0/cam0/data Vocabulary/SIFTvoc.txt -k 10 -L 6 -f 2000\n" << endl;
}

int main(int argc, char **argv)
{
    // 初始化随机数种子
    srand(time(NULL));
    
    cout << "\n========================================" << endl;
    cout << "SIFT Vocabulary Generator for ORB-SLAM2" << endl;
    cout << "========================================\n" << endl;
    
    if(argc < 3)
    {
        printUsage();
        return 1;
    }
    
    // 解析参数
    string strImagePath = string(argv[1]);
    string strOutputFile = string(argv[2]);
    
    int k = 10;          // 分支因子
    int L = 6;           // 树的深度
    int maxFeatures = 2000;  // 每张图像的最大特征数
    
    // 解析可选参数
    for(int i = 3; i < argc; i++)
    {
        if(string(argv[i]) == "-k" && i + 1 < argc)
        {
            k = atoi(argv[++i]);
        }
        else if(string(argv[i]) == "-L" && i + 1 < argc)
        {
            L = atoi(argv[++i]);
        }
        else if(string(argv[i]) == "-f" && i + 1 < argc)
        {
            maxFeatures = atoi(argv[++i]);
        }
    }
    
    cout << "Configuration:" << endl;
    cout << "  Image directory: " << strImagePath << endl;
    cout << "  Output file: " << strOutputFile << endl;
    cout << "  Max features per image: " << maxFeatures << endl;
    cout << "  Vocabulary k: " << k << endl;
    cout << "  Vocabulary L: " << L << endl;
    cout << endl;
    
    // 加载图像路径
    vector<string> vstrImageFilenames;
    loadImages(strImagePath, vstrImageFilenames);
    
    if(vstrImageFilenames.empty())
    {
        cerr << "Error: No images found in " << strImagePath << endl;
        return 1;
    }
    
    // 提取 SIFT 特征
    vector<Mat> features;
    extractSIFTFeatures(vstrImageFilenames, features, maxFeatures);
    
    if(features.empty())
    {
        cerr << "Error: No features extracted" << endl;
        return 1;
    }
    
    // 创建并保存词汇表
    createVocabulary(features, strOutputFile, k, L);
    
    cout << "\n=== Process completed successfully! ===" << endl;
    cout << "Vocabulary saved to: " << strOutputFile << endl;
    cout << "You can now use this vocabulary with ORB-SLAM2 SIFT version\n" << endl;
    
    return 0;
}
