/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "SIFTExtractor.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

const int EDGE_THRESHOLD = 19;

SIFTExtractor::SIFTExtractor(int _nfeatures, float _scaleFactor, int _nlevels,
                             int _iniThFAST, int _minThFAST)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
      iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);

    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    // Create SIFT detector with nfeatures
    sift = cv::SIFT::create(nfeatures);
}

void SIFTExtractor::operator()(InputArray _image, InputArray _mask,
                                std::vector<KeyPoint>& _keypoints,
                                OutputArray _descriptors)
{
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    // Compute pyramid
    ComputePyramid(image);

    std::vector<std::vector<KeyPoint>> allKeypoints;
    allKeypoints.resize(nlevels);

    std::vector<Mat> descriptors;
    descriptors.resize(nlevels);

    // Extract SIFT features on each level
    for (int level = 0; level < nlevels; ++level)
    {
        std::vector<KeyPoint> &keypoints = allKeypoints[level];
        Mat &desc = descriptors[level];

        if(level == 0)
            sift->detectAndCompute(mvImagePyramid[level], _mask, keypoints, desc);
        else
            sift->detectAndCompute(mvImagePyramid[level], Mat(), keypoints, desc);

        // Correct keypoint coordinates and scale
        if(level != 0)
        {
            float scale = mvScaleFactor[level];
            for(auto &kp : keypoints)
            {
                kp.pt *= scale;
                kp.size *= scale;
                kp.octave = level;
            }
        }
        else
        {
            for(auto &kp : keypoints)
            {
                kp.octave = level;
            }
        }
    }

    // Merge all keypoints and descriptors
    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += allKeypoints[level].size();

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    Mat allDescriptors;
    if(nkeypoints > 0)
    {
        allDescriptors = Mat(nkeypoints, 128, CV_32F);
        
        int offset = 0;
        for (int level = 0; level < nlevels; ++level)
        {
            std::vector<KeyPoint> &keypoints = allKeypoints[level];
            Mat &desc = descriptors[level];

            for(size_t j = 0; j < keypoints.size(); j++)
            {
                _keypoints.push_back(keypoints[j]);
                desc.row(j).copyTo(allDescriptors.row(offset));
                offset++;
            }
        }
    }

    _descriptors.create(allDescriptors.rows, allDescriptors.cols, allDescriptors.type());
    allDescriptors.copyTo(_descriptors);
}

void SIFTExtractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);
        }
        else
        {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);
        }
    }
}

} // namespace ORB_SLAM2
