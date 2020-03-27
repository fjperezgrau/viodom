/**
 * @file stereodom.hpp
 * @brief Visual odometry based on the Visual-Inertial Sensor
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

#ifndef __STEREODOM_H__
#define __STEREODOM_H__

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Dynamic Reconfigure Parameters
#include <dynamic_reconfigure/server.h>
#include <viodom/stereodomConfig.h>

#include "imufilter.hpp"
#include "robustmatcher.hpp"

/**
 * @brief Keypoint comparison auxiliar function to sort the sets of keypoints
 * according to their score
 * @param p1 The first keypoint
 * @param p2 The second keypoint
 * @return Boolean true if p1 > p2
 */
bool score_comparator(const cv::KeyPoint& p1, const cv::KeyPoint& p2)
{
    return p1.response > p2.response;
}

/**
 * @class Stereodom
 * @brief Estimates 6DOF pose from stereo image pairs and IMU data
 */
class Stereodom {
public:
    /** @brief Constructor
     * @param nodeName Node name for publishing topics
     * @param leftCam Name of the left camera
     * @param rightCam Name of the right camera
     * @param imuTopic Name of the IMU data topic
     */
    Stereodom(std::string& nodeName, std::string& leftCam, std::string& rightCam, std::string& imuTopic)
        : imu_(0.005, imuTopic, 3.0)
        , it_(nh_)
        , leftImgSubs_(it_, leftCam + "/image_raw", 1)
        , leftInfoSubs_(nh_, leftCam + "/camera_info", 1)
        , rightImgSubs_(it_, rightCam + "/image_raw", 1)
        , rightInfoSubs_(nh_, rightCam + "/camera_info", 1)
        , stereoSync_(leftImgSubs_, rightImgSubs_, 10)
        , cameraInfoSync_(leftInfoSubs_, rightInfoSubs_, 10)
    {
	
        fDetector_ = cv::FastFeatureDetector::create(/*5*/);
        fExtractor_ = cv::xfeatures2d::BriefDescriptorExtractor::create(16 /*32*/);
        std::cout << "Subscribed to:"
                  << "\n";
        std::cout << leftImgSubs_.getTopic() << "\n";
        std::cout << rightImgSubs_.getTopic() << "\n";
        std::cout << leftInfoSubs_.getTopic() << "\n";
        std::cout << rightInfoSubs_.getTopic() << "\n";
        std::cout << imuTopic << "\n";

        //cv::namedWindow("viodom");

        // Get node parameters
        ros::NodeHandle lnh("~");
        if (!lnh.getParam("publish_pc", publishPc_))
            publishPc_ = false;
        if (!lnh.getParam("max_features", maxFeatures_))
            maxFeatures_ = 200;
        if (maxFeatures_ < 100) {
            maxFeatures_ = 200;
            std::cout << "max_features should be above 100. Setting default value (200)" << std::endl;
        }
        if (!lnh.getParam("flow_threshold", flowThreshold_))
            flowThreshold_ = 5.0;
        if (!lnh.getParam("min_matches", minMatches_))
            minMatches_ = 15;
        if (!lnh.getParam("downsampling", downsampling_))
            downsampling_ = 1;
        if (!lnh.getParam("src_frame_id", srcFrameId_))
            srcFrameId_ = "world";
        if (!lnh.getParam("tgt_frame_id", tgtFrameId_))
            tgtFrameId_ = "base_link";

        // Dynamic reconfigure callback
        Stereodom::dyn_rec_f_ = boost::bind(&Stereodom::dynRecCallback, this, _1, _2);
        Stereodom::dyn_rec_server_.setCallback(Stereodom::dyn_rec_f_);

        // Init subscriber synchronizers
        stereoSync_.registerCallback(boost::bind(&Stereodom::stereoCallback, this, _1, _2));
        cameraInfoSync_.registerCallback(boost::bind(&Stereodom::cameraInfoCallback, this, _1, _2));

        // Init publishers
        transformPub_ = nh_.advertise<geometry_msgs::TransformStamped>(nodeName + "/viodom_transform", 1);
        if (publishPc_)
            pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName + "/point_cloud", 1);

        // Init odometry
        odomInit_ = false;
        calibInit_ = false;
        odomC_.setIdentity();
        odomCkf_.setIdentity();
        odom_.setIdentity();

        // Init cam2baselink TFs
        Tcam22imu0_.setOrigin(tf::Vector3(-0.039, 0.0089, 0.0009));
        Tcam22imu0_.setRotation(tf::Quaternion(0.00065, 0.0014, -0.0031, 0.9999));
        Timu02base_.setOrigin(tf::Vector3(0.12, 0.0, -0.06));
        tf::Quaternion q_imu0;
        q_imu0.setRPY(-1.96, 0.0, -1.57);
        Timu02base_.setRotation(q_imu0);
    }

    /** @brief Destructor */
    ~Stereodom(void)
    {
    }

private:
    /** @brief Converts images to OpenCV format and rectify both images
     * @param[in] leftImg Left image in ROS format
     * @param[in] rightImg Right image in ROS format
     * @param[out] imgL Rectified left image in OpenCV format
     * @param[out] imgR Rectified right image in OpenCV format
     */
    bool convertRectifyImages(const sensor_msgs::ImageConstPtr& leftImg,
        const sensor_msgs::ImageConstPtr& rightImg,
        cv::Mat& imgL, cv::Mat& imgR)
    {
        // Convert to OpenCV format without copy
        cv_bridge::CvImageConstPtr cvbLeft, cvbRight;
        try {
            cvbLeft = cv_bridge::toCvShare(leftImg);
            cvbRight = cv_bridge::toCvShare(rightImg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        // Rectify both images
        cv::remap(cvbLeft->image, imgL, mapL1_, mapL2_, cv::INTER_LINEAR);
        cv::remap(cvbRight->image, imgR, mapR1_, mapR2_, cv::INTER_LINEAR);
        return true;
    }

    /** @brief Distribute maxFeatures_ among all buckets.
     * We are considering 6 buckets organized in 2 rows and 3 columns.
     * @param[in] srcKpts Sorted key-points
     * @param[out] dstKpts Output bucketed key-points
     * @param[in] width Image width (cols)
     * @param[in] height Image height (rows)
     */
    void kptsBucketing(std::vector<cv::KeyPoint>& srcKpts, std::vector<cv::KeyPoint>& dstKpts,
        int width, int height)
    {
        const int maxFeatBuck = maxFeatures_ / 6;
        int buckets[6] = { maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck };
        for (size_t i = 0; i < srcKpts.size(); i++) {
            int id;

            // Check keypoint y coord
            if (srcKpts[i].pt.y <= height / 2)
                id = 0;
            else
                id = 3;

            // Check keypoint x coord
            if (srcKpts[i].pt.x <= width / 3)
                id += 0;
            else if (srcKpts[i].pt.x <= 2 * width / 3)
                id += 1;
            else
                id += 2;

            // Assign to corresponding bucket
            if (buckets[id] > 0) {
                buckets[id]--;
                dstKpts.push_back(srcKpts[i]);
            }
        }
    }

    /** @brief Detects key-points in the images
     * Key-points are sorted based on their score to get the best maxFeatures_
     * Features are included in buckets in order to force sparseness
     * @param[in] imgLeft Left image
     * @param[in] imgRight Right image
     * @param[out] keypointsLeft Key-points extracted from left image
     * @param[out] keypointsRight Key-points extracted from right image
     */
    void selectKeypoints(cv::Mat& imgLeft, cv::Mat& imgRight,
        std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight)
    {
        // Detect key-points in both images
        std::vector<cv::KeyPoint> kptsL, kptsR;
        fDetector_->detect(imgLeft, kptsL);
        fDetector_->detect(imgRight, kptsR);

        // Sort keypoints according to their score
        std::sort(kptsL.begin(), kptsL.end(), score_comparator);
        std::sort(kptsR.begin(), kptsR.end(), score_comparator);

        // Distribute maxFeatures_ in buckets
        const int width = imgLeft.cols;
        const int height = imgLeft.rows;
        kptsBucketing(kptsL, keypointsLeft, width, height);
        kptsBucketing(kptsR, keypointsRight, width, height);
    }

    /** @brief Extract feature descriptors from image key-points
     * @param[in] imgLeft Left image
     * @param[in] imgRight Right image
     * @param[in] keypointsLeft Key-points extracted from left image
     * @param[in] keypointsRight Key-points extracted from right image
     * @param[out] descriptorsLeft Feature descriptors for key-points from left image
     * @param[out] descriptorsRight Feature descriptors for key-points from right image
     */
    void extractDescriptors(cv::Mat& imgL, cv::Mat& imgR,
        std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight,
        cv::Mat& descriptorsLeft, cv::Mat& descriptorsRight)
    {
        fExtractor_->compute(imgL, keypointsLeft, descriptorsLeft);
        fExtractor_->compute(imgR, keypointsRight, descriptorsRight);
    }

    /** @brief Match key-points L-R descriptors and generate 3D point cloud
     * @param[in] keypointsLeft Key-points extracted from left image
     * @param[in] keypointsRight Key-points extracted from right image
     * @param[in] descriptorsLeft Feature descriptors for key-points from left image
     * @param[in] descriptorsRight Feature descriptors for key-points from right image
     * @param[out] stereoMatches Resulting matches between features
     * @param[out] pcl Resulting 3D point cloud based on stereo matches between features
     */
    bool stereoMatching(std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight,
        cv::Mat& descriptorsLeft, cv::Mat& descriptorsRight,
        std::vector<cv::DMatch>& stereoMatches, std::vector<cv::Point3f>& pcl)
    {
        std::vector<cv::DMatch> matches;
        try {
            matcher_.match(keypointsRight, descriptorsRight, keypointsLeft, descriptorsLeft,
                matches, false);
        } catch (std::exception e) {
            ROS_ERROR("Matching error!");
            odomInit_ = false;
            return false;
        }

        // Generate stereo point cloud
        float f = PR_.at<double>(0, 0), cx = PR_.at<double>(0, 2), cy = PR_.at<double>(1, 2), b = -PR_.at<double>(0, 3) / f;
        float d, k1 = 1 / f, k2 = f * b;
        for (size_t i = 0; i < matches.size(); i++) {
            const cv::DMatch& match = matches[i];
            const cv::Point2f& pL = keypointsLeft[match.trainIdx].pt;
            const cv::Point2f& pR = keypointsRight[match.queryIdx].pt;
            d = pL.x - pR.x;
            // If key-points y-coords are close enough (stabilized in roll,pitch)
            // and x-coords are far enough (sufficient disparity), it's a match!
            if (fabs(pR.y - pL.y) <= 5 && d > 2.0) {
                stereoMatches.push_back(match);

                // Calculate 3D point
                cv::Point3f p;
                p.z = k2 / d;
                p.x = (pL.x - cx) * p.z * k1;
                p.y = (pL.y - cy) * p.z * k1;
                pcl.push_back(p);
            }
        }
        return true;
    }

    /** @brief Compute feature flow between matched key-points
     * @param matches List of matches
     * @param trainKpts Key-points from 'train' image
     * @param queryKpts Key-points fraom 'query' image
     * @return Computed feature flow
     */
    double computeFeatureFlow(const std::vector<cv::DMatch>& matches,
        const std::vector<cv::KeyPoint>& trainKpts,
        const std::vector<cv::KeyPoint>& queryKpts)
    {
        int idT, idQ;
        double xDiff, yDiff, totalFlow = 0.0;

        for (size_t i = 0; i < matches.size(); i++) {
            idT = matches[i].trainIdx;
            idQ = matches[i].queryIdx;
            xDiff = trainKpts[idT].pt.x - queryKpts[idQ].pt.x;
            yDiff = trainKpts[idT].pt.y - queryKpts[idQ].pt.y;
            totalFlow += sqrt(xDiff * xDiff + yDiff * yDiff);
        }

        return totalFlow / matches.size();
    }

    /** @brief Match key-points left prev-curr descriptors and generate 3D point cloud
     * @param[in] keypointsCurr Key-points extracted from current left image
     * @param[in] keypointsPrev Key-points extracted from previous left image
     * @param[in] descriptorsCurr Feature descriptors for key-points from current left image
     * @param[in] descriptorsPrev Feature descriptors for key-points from previous left image
     * @param[in] stereoMatchesPrev Matches between features from previous left image
     * @param[in] pclPrev 3D point cloud based on stereo matches from previous left image
     * @param[out] flow Feature flow between previous and current left images
     * @param[out] points3d 3D points matched between previous and current left images, taken from pclPrev
     * @param[out] projections 2D projections of matched 3D points in current left image
     */
    bool leftMatching(std::vector<cv::KeyPoint>& keypointsCurr,
        std::vector<cv::KeyPoint>& keypointsPrev,
        cv::Mat& descriptorsCurr, cv::Mat& descriptorsPrev,
        std::vector<cv::DMatch>& stereoMatchesPrev, std::vector<cv::Point3f>& pclPrev,
        double& flow, std::vector<cv::Point3f>& points3d,
        std::vector<cv::Point2f>& projections)
    {
        std::vector<cv::DMatch> matches;
        try {
            matcher_.match(keypointsCurr, descriptorsCurr, keypointsPrev, descriptorsPrev, matches);
        } catch (std::exception e) {
            ROS_ERROR("Matching error!");
            odomInit_ = false;
            return false;
        }

        // Compute features flow
        flow = computeFeatureFlow(matches, keypointsPrev, keypointsCurr);

        // Establish matching between previous and current left images
        for (size_t i = 0; i < stereoMatchesPrev.size(); i++) {
            int idLP = stereoMatchesPrev[i].trainIdx;
            for (size_t j = 0; j < matches.size(); j++) {
                if (matches[j].trainIdx == idLP) {
                    points3d.push_back(pclPrev[i]);
                    projections.push_back(keypointsCurr[matches[j].queryIdx].pt);
                    break;
                }
            }
        }

        //std::cout << "Keypoints: current (" << kptsLeftC_.size() << "), previous (" << kptsLeftP_.size() << ")\n";
        //std::cout << "Previous matches: " << leftMatches.size() << " (flow " << flow << ") -> " << points3d.size() << std::endl;
        return true;
    }

    /** @brief Converts OpenCV translation and rotation into tf::Transform
     * @param[in] t OpenCV translation vector
     * @param[in] R OpenCV rotation matrix
     * @param[out] transform ROS tf::Transform element
     */
    void openCVToTf(const cv::Mat& t, const cv::Mat& R, tf::Transform& tf)
    {
        tf::Vector3 translationTf(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));

        tf::Matrix3x3 rotationTf;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rotationTf[i][j] = R.at<double>(i, j);

        tf.setOrigin(translationTf);
        tf.setBasis(rotationTf);
    }

    /** @brief Converts OpenCV translation and rotation into tf::Transform
     * @param[in] t OpenCV translation vector
     * @param[in] R OpenCV rotation matrix
     * @param[out] transform ROS tf::Transform element
     */
    void tfToOpenCV(const tf::Transform& tf, cv::Mat& t, cv::Mat& R)
    {
        t.at<double>(0, 0) = tf.getOrigin().getX();
        t.at<double>(1, 0) = tf.getOrigin().getY();
        t.at<double>(2, 0) = tf.getOrigin().getZ();

        tf::Matrix3x3 rotationTf = tf.getBasis();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R.at<double>(i, j) = rotationTf[i][j];
    }

    /** @brief Solve PnP problem between previous and current left images
     * @param[out] points3d 3D points matched between previous and current left images, taken from previous 3D point cloud
     * @param[out] projections 2D projections of matched 3D points in current left image
     * @param[out] deltaT Estimated transform between previous and current left frames
     */
    bool leftPnP(std::vector<cv::Point3f>& points3d, std::vector<cv::Point2f>& projections,
        tf::Transform& deltaT)
    {
        cv::Mat R, T, inliers;
        T = cv::Mat::zeros(3, 1, CV_64FC1);

        // Use IMU angles since last key-frame as initial approximation of rotation in solvePnP
        tf::Matrix3x3 dR;
        double rx, ry, rz;
        imu_.getAngleIntegration(rx, ry, rz);
        dR.setRPY(-ry, -rz, -rx);
        cv::Mat rot1(3, 3, CV_64FC1), rot2;
        rot1.at<double>(0, 0) = dR[0][0];
        rot1.at<double>(0, 1) = dR[0][1];
        rot1.at<double>(0, 2) = dR[0][2];
        rot1.at<double>(1, 0) = dR[1][0];
        rot1.at<double>(1, 1) = dR[1][1];
        rot1.at<double>(1, 2) = dR[1][2];
        rot1.at<double>(2, 0) = dR[2][0];
        rot1.at<double>(2, 1) = dR[2][1];
        rot1.at<double>(2, 2) = dR[2][2];
        cv::Rodrigues(rot1, rot2);

        // Solve PnP using RANSAC and EPNP algorithm
        try {
            cv::solvePnPRansac(points3d, projections, KL_, cv::Mat(), rot2, T, true, 150, 2, 100, inliers, CV_EPNP);
            cv::Rodrigues(rot2, R);
        } catch (std::exception e) {
            ROS_ERROR("PnP error!");
            odomInit_ = false;
            return false;
        }

        // Convert to tf::Transform
        openCVToTf(T, R, deltaT);

        return true;
    }

    /** @brief Compensate IMU roll and pitch angles in base_link frame
     */
    void compensateIMU(tf::Transform& odom)
    {
        double rxImu, ryImu, rzImu, rxOdom, ryOdom, rzOdom;
        imu_.getAngles(rxImu, ryImu, rzImu);
        tf::Matrix3x3(odom.getRotation()).getRPY(rxOdom, ryOdom, rzOdom);
        tf::Quaternion qCompensated;
        qCompensated.setRPY(rxImu, -(ryImu + 0.4), rzOdom);
        odom.setRotation(qCompensated);
    }

    /** @brief Convert odometry from camera frame to the robot base_link frame
     * @param odomC Odometry transform in camera frame
     * @return Odometry transform in the robot base_link frame
     */
    tf::Transform camera2baselink(const tf::Transform& odomC)
    {
        tf::Transform odom = Timu02base_ * Tcam22imu0_ * odomC * Tcam22imu0_.inverse() * Timu02base_.inverse();
        return odom;
    }

    /** @brief Convert odometry from the robot base_link frame to camera frame
     * @param odom Odometry transform in base_link frame
     * @return Odometry transform in camera frame
     */
    tf::Transform baselink2camera(const tf::Transform& odom)
    {
        tf::Transform odomC = Tcam22imu0_.inverse() * Timu02base_.inverse() * odom * Timu02base_ * Tcam22imu0_;
        return odomC;
    }

    /** @brief Publish 6DOF pose as TF and geometry_msgs::TransformStamped
     * @param odom Odometry transform to be published
     */
    void publishTf(const tf::Transform& odom)
    {
        // Publish TF
        tfBr_.sendTransform(
            tf::StampedTransform(odom, ros::Time::now(),
                srcFrameId_, tgtFrameId_));
        // Publish pose
        geometry_msgs::TransformStamped outTransform;
        outTransform.header.frame_id = srcFrameId_;
        outTransform.header.stamp = ros::Time::now();
        tf::Quaternion qPose = odom.getRotation();
        outTransform.transform.rotation.x = qPose.x();
        outTransform.transform.rotation.y = qPose.y();
        outTransform.transform.rotation.z = qPose.z();
        outTransform.transform.rotation.w = qPose.w();
        tf::Vector3 pPose = odom.getOrigin();
        outTransform.transform.translation.x = pPose.x();
        outTransform.transform.translation.y = pPose.y();
        outTransform.transform.translation.z = pPose.z();
        transformPub_.publish(outTransform);
    }

    /** @brief Publish 3D point cloud
     * @param[in] header Header for the point cloud message
     */
    void publishPointCloud(const std::vector<cv::Point3f> pcl, const std_msgs::Header header)
    {
        // Fill PointCloud message
        sensor_msgs::PointCloud outCloud;
        outCloud.points.resize(pcl.size());
        outCloud.header = header;
        for (size_t i = 0; i < outCloud.points.size(); i++) {
            outCloud.points[i].x = pcl[i].x;
            outCloud.points[i].y = pcl[i].y;
            outCloud.points[i].z = pcl[i].z;
        }

        // Convert PointCloud to PointCloud2
        sensor_msgs::PointCloud2 outCloud2;
        sensor_msgs::convertPointCloudToPointCloud2(outCloud, outCloud2);

        // Publish point cloud
        pcPub_.publish(outCloud2);
    }

    /** @brief Update previous-current keypoints, images, descriptors, matches and point clouds
     * @param[in] imgL Current left image
     * @param[in] imgR Current right image
     */
    void updatePreviousStuff(const cv::Mat& imgL, const cv::Mat& imgR)
    {
        // Save current images as previous ones for next comparison
        imgL.copyTo(imgLP_);
        imgR.copyTo(imgRP_);

        // Save current key-points and descriptors
        kptsLeftP_ = kptsLeftC_;
        kptsRightP_ = kptsRightC_;
        descLeftC_.copyTo(descLeftP_);
        descRightC_.copyTo(descRightP_);

        // Save stereo matching results, 2D projections and 3D point cloud
        stereoMatchesP_ = stereoMatchesC_;
        pclP_ = pclC_;

        // Reset angle rate integration
        imu_.resetAngleIntegration();
    }

    /** @brief Stereo callback with image rectification
     * @param leftImg Left image
     * @param rightImg Right image
     */
    void stereoCallback(const sensor_msgs::ImageConstPtr& leftImg,
        const sensor_msgs::ImageConstPtr& rightImg)
    {
        double flow = 0.0;

        // Processing does not start until first calibration is received
        if (!calibInit_) {
            ROS_WARN("calibInit = false, exiting callback");
            return;
        }

        // Processing does not start until imu_ is initialized
        if (!imu_.isInit()) {
            ROS_WARN("imu.isInit = false, exiting callback");
            return;
        }

        // Convert to OpenCV format and rectify both images
        cv::Mat imgL, imgR;
        if (!convertRectifyImages(leftImg, rightImg, imgL, imgR))
            return;

        // Detect key-points in the images
        kptsLeftC_.clear();
        kptsRightC_.clear();
        selectKeypoints(imgL, imgR, kptsLeftC_, kptsRightC_);
        //std::cout << "Keypoints: left (" << kptsLeftC_.size() << "), right (" << kptsRightC_.size() << ")\n";

        // Extract feature descriptors
        extractDescriptors(imgL, imgR, kptsLeftC_, kptsRightC_, descLeftC_, descRightC_);

        // Stereo matching and 3D point cloud generation
        pclC_.clear();
        stereoMatchesC_.clear();
        if (!stereoMatching(kptsLeftC_, kptsRightC_, descLeftC_, descRightC_, stereoMatchesC_, pclC_))
            return;
        //std::cout << "Stereo matches: " stereoMatchesC_.size() << std::endl;

        // If not the first frame, compute odometry
        if (odomInit_) {
            // Matching between previous left key-frame and current left image
            std::vector<cv::Point3f> points3d;
            std::vector<cv::Point2f> projections;
            if (!leftMatching(kptsLeftC_, kptsLeftP_, descLeftC_, descLeftP_,
                    stereoMatchesP_, pclP_, flow, points3d, projections))
                return;

            // If there are enough matches
            if (points3d.size() >= minMatches_) {
                // Compute PnP between previous key-frame and current frame
                tf::Transform deltaT;
                if (!leftPnP(points3d, projections, deltaT))
                    return;

                odomC_ = odomCkf_ * deltaT.inverse();
                imgL.copyTo(imgC_);
                odom_ = camera2baselink(odomC_);

                // Compensate IMU roll and pitch angles in base_link frame
                compensateIMU(odom_);

                // Transform back to camera frame to compensate odomC_ accordingly
                odomC_ = baselink2camera(odom_);

                if (flow > flowThreshold_) {
                    // Save key-frame transform and image
                    odomCkf_ = odomC_;
                    imgL.copyTo(imgCkf_);
                }

                //Publish transform
                publishTf(odom_);

                // Publish current 3D point cloud
                if (publishPc_)
                    publishPointCloud(pclC_, leftImg->header);
            } else
                ROS_WARN_STREAM("Not enough matches! (" << points3d.size() << ")");
        } else
            ROS_WARN("odomInit = false");

        // If new key-frame or first image pair
        if (flow > flowThreshold_ || !odomInit_) {
            updatePreviousStuff(imgL, imgR);
        }

        // Set the init flag to true
        odomInit_ = true;
    }

    /** @brief Camera calibration info callback
     * @param leftInfo Left camera calibration data
     * @param rightInfo Right camera calibration data
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& leftInfo,
        const sensor_msgs::CameraInfoConstPtr& rightInfo)
    {
        if (!calibInit_) {
            // Store stereo camera parameters
            RL_ = cv::Mat(3, 3, CV_64FC1, (void*)leftInfo->R.elems).clone();
            PL_ = cv::Mat(3, 4, CV_64FC1, (void*)leftInfo->P.elems).clone();
            RR_ = cv::Mat(3, 3, CV_64FC1, (void*)rightInfo->R.elems).clone();
            PR_ = cv::Mat(3, 4, CV_64FC1, (void*)rightInfo->P.elems).clone();

            // Obtain the corresponding PL and PR of the downsampled images
            PL_.at<double>(0, 0) = PL_.at<double>(0, 0) / (float)downsampling_;
            PL_.at<double>(0, 2) = PL_.at<double>(0, 2) / (float)downsampling_;
            PL_.at<double>(0, 3) = PL_.at<double>(0, 3) / (float)downsampling_;
            PL_.at<double>(1, 1) = PL_.at<double>(1, 1) / (float)downsampling_;
            PL_.at<double>(1, 2) = PL_.at<double>(1, 2) / (float)downsampling_;
            PR_.at<double>(0, 0) = PR_.at<double>(0, 0) / (float)downsampling_;
            PR_.at<double>(0, 2) = PR_.at<double>(0, 2) / (float)downsampling_;
            PR_.at<double>(0, 3) = PR_.at<double>(0, 3) / (float)downsampling_;
            PR_.at<double>(1, 1) = PR_.at<double>(1, 1) / (float)downsampling_;
            PR_.at<double>(1, 2) = PR_.at<double>(1, 2) / (float)downsampling_;

            // Initialize left and right image remapping (rectification and undistortion)
            cv::initUndistortRectifyMap(cv::Mat(3, 3, CV_64FC1, (void*)leftInfo->K.elems),
                cv::Mat(4, 1, CV_64FC1, (void*)leftInfo->D.data()),
                RL_, PL_,
                cv::Size(leftInfo->width / (float)downsampling_, leftInfo->height / (float)downsampling_),
                CV_16SC2, mapL1_, mapL2_);
            cv::initUndistortRectifyMap(cv::Mat(3, 3, CV_64FC1, (void*)rightInfo->K.elems),
                cv::Mat(4, 1, CV_64FC1, (void*)rightInfo->D.data()),
                RR_, PR_,
                cv::Size(rightInfo->width / (float)downsampling_, rightInfo->height / (float)downsampling_),
                CV_16SC2, mapR1_, mapR2_);

            // Store single camera calibration after rectification (no distortion)
            KL_ = cv::Mat(3, 3, CV_64FC1);
            KL_.at<double>(0, 0) = PL_.at<double>(0, 0);
            KL_.at<double>(0, 1) = PL_.at<double>(0, 1);
            KL_.at<double>(0, 2) = PL_.at<double>(0, 2);
            KL_.at<double>(1, 0) = PL_.at<double>(1, 0);
            KL_.at<double>(1, 1) = PL_.at<double>(1, 1);
            KL_.at<double>(1, 2) = PL_.at<double>(1, 2);
            KL_.at<double>(2, 0) = PL_.at<double>(2, 0);
            KL_.at<double>(2, 1) = PL_.at<double>(2, 1);
            KL_.at<double>(2, 2) = PL_.at<double>(2, 2);
            KR_ = cv::Mat(3, 3, CV_64FC1);
            KR_.at<double>(0, 0) = PR_.at<double>(0, 0);
            KR_.at<double>(0, 1) = PR_.at<double>(0, 1);
            KR_.at<double>(0, 2) = PR_.at<double>(0, 2);
            KR_.at<double>(1, 0) = PR_.at<double>(1, 0);
            KR_.at<double>(1, 1) = PR_.at<double>(1, 1);
            KR_.at<double>(1, 2) = PR_.at<double>(1, 2);
            KR_.at<double>(2, 0) = PR_.at<double>(2, 0);
            KR_.at<double>(2, 1) = PR_.at<double>(2, 1);
            KR_.at<double>(2, 2) = PR_.at<double>(2, 2);

            // Set calibration flag to true
            calibInit_ = true;
        }
    }

    /** @brief Dynamic reconfigure callback
     */
    void dynRecCallback(viodom::stereodomConfig& config, uint32_t level)
    {
        ROS_INFO("Reconfiguring");
        downsampling_ = config.downsampling;
        maxFeatures_ = config.max_features;
        flowThreshold_ = config.flow_threshold;
        minMatches_ = config.min_matches;
    }

    //void showMatchesImage(const std::vector<cv::DMatch>& matches)
    //{
    //    cv::Mat image;
    //    cv::drawMatches(imgC, kptsLeftC, imgLP, kptsLeftP, matches, image);
    //    cv::imshow("viodom", image);
    //    cv::waitKey(5);
    //}

    std::vector<cv::KeyPoint> kptsLeftP_, kptsLeftC_; /**< Stored previous (P) and current (C) keypoints for left image*/
    std::vector<cv::KeyPoint> kptsRightP_, kptsRightC_; /**< Stored previous (P) and current (C) keypoints for right image*/
    cv::Mat descLeftP_, descLeftC_; /**< Stored previous (P) and current (C) descriptors for left image*/
    cv::Mat descRightP_, descRightC_; /**< Stored previous (P) and current (C) descriptors for right image*/

    cv::Ptr<cv::FastFeatureDetector> fDetector_; /**< Feature detector (FAST)*/
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> fExtractor_; /**< Feature decriptor extractor (BRIEF)*/

    RobustMatcher matcher_; /**< Matcher*/

    std::vector<cv::DMatch> stereoMatchesP_, stereoMatchesC_; /**< Matching results for previous(P) and current(C) stereo pairs*/
    std::vector<cv::Point3f> pclP_, pclC_; /**< Stored previous (P) and current(C) 3D point clouds*/

    ros::NodeHandle nh_; /**< ROS node handler*/
    image_transport::ImageTransport it_; /**< Image transport*/
    image_transport::SubscriberFilter leftImgSubs_, rightImgSubs_; /**< Image subscribers*/
    message_filters::Subscriber<sensor_msgs::CameraInfo> leftInfoSubs_, rightInfoSubs_; /**< Camera info subscribers*/

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereoSync_; /**< Time synchronizer filter for images*/
    message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> cameraInfoSync_; /**< Time synchronizer filter for camera info*/

    ros::Publisher transformPub_; /**< Publisher for output odometry transform*/
    ros::Publisher pcPub_; /**< Publisher for 3D point cloud*/

    bool odomInit_; /**< Flag to determine whether to perform odometry or not*/
    bool calibInit_; /**< Flag indicating if we have calibration data*/

    // Node params
    bool publishPc_; /**< Flag to determine whether to publish 3D point cloud*/
    int maxFeatures_; /**< Maximum number of features to find in each bucket*/
    double flowThreshold_; /**< Minimum feature flow to determine if a new key-frame has to be created*/
    int minMatches_; /**< Minimum number of matches to perform odometry estimation*/
    int downsampling_; /**< Downsampling factor for the input images*/
    std::string srcFrameId_; /**< Source frame for odometry transformation*/
    std::string tgtFrameId_; /**< Target frame for odometry transformation*/
    tf::Transform Tcam22imu0_, Timu02base_; /**< Auxiliary transforms between camera and base_link frames*/
    double kfTrTh_, kfRotTh_; /**< Thresholds to trigger key-frame creation for SBA (translation in meters, rotation in radians)*/

    cv::Mat KL_, PL_, RL_; /**< Stereo camera parameters for left (L) camera*/
    cv::Mat KR_, PR_, RR_; /**< Stereo camera parameters for right (R) camera*/

    cv::Mat mapL1_, mapL2_, mapR1_, mapR2_; /**< Stereo rectification mappings*/

    tf::Transform odom_, odomC_, odomCkf_; /**< Odometry matrices in camera and base_link frames*/
    cv::Mat imgC_, imgCkf_; /**< Current and key-frame images*/
    cv::Mat imgRP_, imgLP_; /**< Previous right(R) and left(L) images*/

    tf::TransformBroadcaster tfBr_; /**< Transform broadcaster*/

    ImuFilter imu_; /**< IMU processor*/

    dynamic_reconfigure::Server<viodom::stereodomConfig> dyn_rec_server_; /**< Dynamic reconfigure server*/
    dynamic_reconfigure::Server<viodom::stereodomConfig>::CallbackType dyn_rec_f_; /**< Dynamic reconfigure callback*/
};

#endif
