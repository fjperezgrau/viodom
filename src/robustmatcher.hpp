/**
 * @file robustmatcher.hpp
 * @brief Robust feature matcher between two sets of key-points and descriptors
 *
 * Partly based on the work included in 'OpenCV 2 Computer Vision Application Programming Cookbook'
 * https://www.packtpub.com/application-development/opencv-2-computer-vision-application-programming-cookbook
 *
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 *
Copyright (c) 2016, fcaballero, fjperezgrau
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ROBUSTMATCHER_H__
#define __ROBUSTMATCHER_H__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/**
 * @class RobustMatcher
 * @brief Matches two sets of key-points and descriptors
 */
class RobustMatcher
{
public:
    /** @brief Constructor
     */
    RobustMatcher(void):
        matcher_(cv::NORM_HAMMING)
	{	
		// Variable initialization
        ratio_ = 0.65f;
        refineF_ = false;
        confidence_ = 0.99;
        distance_ = 3.0;
	}
	
    /** @brief Match feature points using symmetry test and RANSAC
     * @param[in] keypoints1 Set of key-points from first image
     * @param[in] descriptors1 Set of feature descriptors from first image
     * @param[in] keypoints2 Set of key-points from second image
     * @param[in] descriptors2 Set of feature descriptors from second image
     * @param[out] matches Resulting robust matches between the first and second images
     * @param[in] doRansacTest If true, perform RANSAC test to matches
     * @param[in] doRatioTest If true, perform ratio test to matches
     * @param[in] refineF If true, refine fundamental matrix
     * @return Fundamental matrix
     */
    cv::Mat match(std::vector<cv::KeyPoint>& keypoints1, cv::Mat &descriptors1,
                  std::vector<cv::KeyPoint>& keypoints2, cv::Mat &descriptors2,
                  std::vector<cv::DMatch>& matches, bool doRansacTest = true,
                  bool doRatioTest = false, bool refineF = false)
	{
		cv::Mat F;
		
        // Matching from image 1 to image 2 based on k nearest neighbours (k=2)
        std::vector<std::vector<cv::DMatch> > matches12;
        matcher_.knnMatch(descriptors1, descriptors2, matches12, 2);
		
        // Matching from image 2 to image 1 based on k nearest neighbours (k=2)
        std::vector<std::vector<cv::DMatch> > matches21;
        matcher_.knnMatch(descriptors2, descriptors1, matches21, 2);
		
        // Remove matches for which NN ratio > threshold
        if(doRatioTest)
		{
            ratioTest(matches12);
            ratioTest(matches21);
		}
		
        // Remove non-symmetrical matches
		std::vector<cv::DMatch> symMatches;
        symmetryTest(matches12, matches21, symMatches);
		
        // Validate matches using RANSAC
        if(doRansacTest)
		{
            refineF_ = refineF;
			F = ransacTest(symMatches, keypoints1, keypoints2, matches);
		}
		else
			matches = symMatches;
		
		return F;
	}

    /** @brief Clear matches for which NN ratio is greater than threshold
     * The corresponding entries in the vector of matches are cleared
     * @param matches Vector of matches
     * @return Number of removed points
     */
	int ratioTest(std::vector<std::vector<cv::DMatch> >	&matches) 
	{
		int removed=0;
        for(std::vector<std::vector<cv::DMatch> >::iterator it = matches.begin(); it!= matches.end(); ++it)
		{
            // if 2 nearest neighbors have been identified
            if(it->size() > 1)
			{
				// check distance ratio
                if((*it)[0].distance/(*it)[1].distance > ratio_)
				{
                    it->clear(); // remove match
					removed++;
				}
			} 
			else 
            {   // does not have 2 neighbors
                it->clear(); // remove match
				removed++;
			}
		}
		return removed;
	}

    /** @brief Checks that matches are present both ways
     * @param[in] matches1 Matched points in one direction
     * @param[in] matches2 Matched points in the opposite direction
     * @param[out] symMatches Output symmetrical matches
     */
    void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                      const std::vector<std::vector<cv::DMatch> >& matches2,
                      std::vector<cv::DMatch>& symMatches)
	{
		// for all matches image 1 -> image 2
        for(std::vector<std::vector<cv::DMatch> >::const_iterator it1= matches1.begin(); it1!= matches1.end(); ++it1)
		{
			// ignore deleted matches
            if(it1->size() < 2)
				continue;
				
			// for all matches image 2 -> image 1
            for(std::vector<std::vector<cv::DMatch> >::const_iterator it2= matches2.begin(); it2!= matches2.end(); ++it2)
			{
				// ignore deleted matches
                if(it2->size() < 2)
					continue;
				// Match symmetry test
                if((*it1)[0].queryIdx ==(*it2)[0].trainIdx &&
                        (*it2)[0].queryIdx == (*it1)[0].trainIdx)
				{
					// add symmetrical match
                    symMatches.push_back(cv::DMatch((*it1)[0].queryIdx, (*it1)[0].trainIdx, (*it1)[0].distance));
					break; // next match in image 1 -> image 2
				}
			}
		}
	}
	
    /** @brief Identify good matches using RANSAC
     * @param[in] matches Input matches between images 1 and 2
     * @param[in] keypoints1 Key-points from image 1
     * @param[in] keypoints2 Key-points from image 2
     * @param[out] outMatches Surviving matches after RANSAC test
     * @return Fundamental matrix
     */
    cv::Mat ransacTest(const std::vector<cv::DMatch>& matches,
                       const std::vector<cv::KeyPoint>& keypoints1,
                       const std::vector<cv::KeyPoint>& keypoints2,
                       std::vector<cv::DMatch>& outMatches)
	{
		cv::Mat nullMat;
		
        // Convert keypoints into Point2f
		std::vector<cv::Point2f> points1, points2;		
        for(std::vector<cv::DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it)
		{
			// Get the position of left keypoints
            float x = keypoints1[it->queryIdx].pt.x;
            float y = keypoints1[it->queryIdx].pt.y;
			points1.push_back(cv::Point2f(x,y));

			// Get the position of right keypoints
            x = keypoints2[it->trainIdx].pt.x;
            y = keypoints2[it->trainIdx].pt.y;
			points2.push_back(cv::Point2f(x,y));
		}
		
		// Compute F matrix using RANSAC
		if(points1.size() < 10)
			return nullMat;
		std::vector<uchar> inliers(points1.size(),0);
        cv::Mat fundamental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), inliers,
                                                    CV_FM_RANSAC, distance_, confidence_);
		
        // Extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator itIn = inliers.begin();
        std::vector<cv::DMatch>::const_iterator itM = matches.begin();
		
		// for all matches
        for(; itIn!= inliers.end(); ++itIn, ++itM)
		{
            if(*itIn)
			{ // it is a valid match
				outMatches.push_back(*itM);
			}
		}
		
        if (refineF_)
		{
            // The F matrix will be recomputed with all accepted matches
            // Convert surviving keypoints into Point2f for final F computation
			points1.clear();
			points2.clear();
            for(std::vector<cv::DMatch>::const_iterator it = outMatches.begin(); it!= outMatches.end(); ++it)
			{
				// Get the position of left keypoints
                float x = keypoints1[it->queryIdx].pt.x;
                float y = keypoints1[it->queryIdx].pt.y;
				points1.push_back(cv::Point2f(x,y));

				// Get the position of right keypoints
                x = keypoints2[it->trainIdx].pt.x;
                y = keypoints2[it->trainIdx].pt.y;
				points2.push_back(cv::Point2f(x,y));
			}

			// Compute 8-point F from all accepted matches
			if(points1.size() < 9)
				return nullMat;
            fundamental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), CV_FM_8POINT);
		}
		
        return fundamental;
	}

private:

    float ratio_;           /**< Max ratio between 1st and 2nd nearest neighbor*/
    bool refineF_;          /**< If true the F matrix will be refined*/
    double distance_;       /**< Min distance to epipolar in RANSAC*/
    double confidence_;     /**< Confidence level (probability)*/
    cv::BFMatcher matcher_; /**< Bruteforce matcher*/

};

#endif
