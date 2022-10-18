// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modified by: Rafael Goulart        rgoulartns@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "livox_loam/livox_loam.hpp"

// TODO: https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html

namespace livox_loam
{

  ScanRegistration::ScanRegistration() :
    Node("scan_registration_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
    matA1(3, 3, CV_32F, cv::Scalar::all(0)),
    matD1(1, 3, CV_32F, cv::Scalar::all(0)),
    matV1(3, 3, CV_32F, cv::Scalar::all(0))
  {

    pCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/cloud",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ScanRegistration::pCloudCB, this, std::placeholders::_1));

    pCloudCorner_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/cloud/corner", 1);
    pCloudSurf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/cloud/surf", 1);
  }

  ScanRegistration::~ScanRegistration() {}

  void ScanRegistration::pCloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudInMsg)
  {
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(*cloudInMsg, laserCloudIn);

    int cloudSize = laserCloudIn.points.size();

    if (cloudSize > 32000)
      cloudSize = 32000;

    pcl::PointXYZI point;
    pcl::PointCloud<pcl::PointXYZI> Allpoints;

    for (int i = 0; i < cloudSize; i++)
    {
      point.x = laserCloudIn.points[i].x;
      point.y = laserCloudIn.points[i].y;
      point.z = laserCloudIn.points[i].z;
      double theta = std::atan2(laserCloudIn.points[i].y, laserCloudIn.points[i].z) / M_PI * 180 + 180;

      int scanID = std::floor(theta / 9);
      point.intensity = scanID + (laserCloudIn.points[i].intensity / 10000);
      // point.intensity = scanID+(double(i)/cloudSize);

      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        continue;

      Allpoints.push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    *laserCloud += Allpoints;
    cloudSize = laserCloud->size();

    for (int i = 0; i < cloudSize; i++)
      CloudFeatureFlag[i] = 0;

    int count_num = 1;
    bool left_surf_flag = false;
    bool right_surf_flag = false;
    Eigen::Vector3d surf_vector_current(0, 0, 0);
    Eigen::Vector3d surf_vector_last(0, 0, 0);
    //********************************************************************************************************************************************
    for (int i = 5; i < cloudSize - 5; i += count_num)
    {
      // left curvature
      float ldiffX = laserCloud->points[i - 4].x + laserCloud->points[i - 3].x - 4 * laserCloud->points[i - 2].x + laserCloud->points[i - 1].x + laserCloud->points[i].x;
      float ldiffY = laserCloud->points[i - 4].y + laserCloud->points[i - 3].y - 4 * laserCloud->points[i - 2].y + laserCloud->points[i - 1].y + laserCloud->points[i].y;
      float ldiffZ = laserCloud->points[i - 4].z + laserCloud->points[i - 3].z - 4 * laserCloud->points[i - 2].z + laserCloud->points[i - 1].z + laserCloud->points[i].z;
      float left_curvature = ldiffX * ldiffX + ldiffY * ldiffY + ldiffZ * ldiffZ;

      if (left_curvature < 0.01)
      {
        std::vector<pcl::PointXYZI> left_list;

        for (int j = -4; j < 0; j++)
          left_list.push_back(laserCloud->points[i + j]);

        if (left_curvature < 0.001)
          CloudFeatureFlag[i - 2] = 1; // surf point flag  && plane_judge(left_list,1000)

        left_surf_flag = true;
      }
      else
      {
        left_surf_flag = false;
      }

      // right curvature
      float rdiffX = laserCloud->points[i + 4].x + laserCloud->points[i + 3].x - 4 * laserCloud->points[i + 2].x + laserCloud->points[i + 1].x + laserCloud->points[i].x;
      float rdiffY = laserCloud->points[i + 4].y + laserCloud->points[i + 3].y - 4 * laserCloud->points[i + 2].y + laserCloud->points[i + 1].y + laserCloud->points[i].y;
      float rdiffZ = laserCloud->points[i + 4].z + laserCloud->points[i + 3].z - 4 * laserCloud->points[i + 2].z + laserCloud->points[i + 1].z + laserCloud->points[i].z;
      float right_curvature = rdiffX * rdiffX + rdiffY * rdiffY + rdiffZ * rdiffZ;

      if (right_curvature < 0.01)
      {
        std::vector<pcl::PointXYZI> right_list;

        for (int j = 1; j < 5; j++)
          right_list.push_back(laserCloud->points[i + j]);

        if (right_curvature < 0.001)
          CloudFeatureFlag[i + 2] = 1; // surf point flag  && plane_judge(right_list,1000)

        count_num = 4;
        right_surf_flag = true;
      }
      else
      {
        count_num = 1;
        right_surf_flag = false;
      }

      // surf-surf corner feature
      if (left_surf_flag && right_surf_flag)
      {
        Eigen::Vector3d norm_left(0, 0, 0);
        Eigen::Vector3d norm_right(0, 0, 0);
        for (int k = 1; k < 5; k++)
        {
          Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i - k].x - laserCloud->points[i].x,
                                                laserCloud->points[i - k].y - laserCloud->points[i].y,
                                                laserCloud->points[i - k].z - laserCloud->points[i].z);
          tmp.normalize();
          norm_left += (k / 10.0) * tmp;
        }
        for (int k = 1; k < 5; k++)
        {
          Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i + k].x - laserCloud->points[i].x,
                                                laserCloud->points[i + k].y - laserCloud->points[i].y,
                                                laserCloud->points[i + k].z - laserCloud->points[i].z);
          tmp.normalize();
          norm_right += (k / 10.0) * tmp;
        }

        // calculate the angle between this group and the previous group
        double cc = fabs(norm_left.dot(norm_right) / (norm_left.norm() * norm_right.norm()));
        // calculate the maximum distance, the distance cannot be too small
        Eigen::Vector3d last_tmp = Eigen::Vector3d(laserCloud->points[i - 4].x - laserCloud->points[i].x,
                                                   laserCloud->points[i - 4].y - laserCloud->points[i].y,
                                                   laserCloud->points[i - 4].z - laserCloud->points[i].z);
        Eigen::Vector3d current_tmp = Eigen::Vector3d(laserCloud->points[i + 4].x - laserCloud->points[i].x,
                                                      laserCloud->points[i + 4].y - laserCloud->points[i].y,
                                                      laserCloud->points[i + 4].z - laserCloud->points[i].z);
        double last_dis = last_tmp.norm();
        double current_dis = current_tmp.norm();

        if (cc < 0.5 && last_dis > 0.05 && current_dis > 0.05)
          CloudFeatureFlag[i] = 150;
      }
    }
    for (int i = 5; i < cloudSize - 5; i++)
    {
      float diff_left[2];
      float diff_right[2];
      float depth = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                         laserCloud->points[i].y * laserCloud->points[i].y +
                         laserCloud->points[i].z * laserCloud->points[i].z);

      for (int count = 1; count < 3; count++)
      {
        float diffX1 = laserCloud->points[i + count].x - laserCloud->points[i].x;
        float diffY1 = laserCloud->points[i + count].y - laserCloud->points[i].y;
        float diffZ1 = laserCloud->points[i + count].z - laserCloud->points[i].z;
        diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

        float diffX2 = laserCloud->points[i - count].x - laserCloud->points[i].x;
        float diffY2 = laserCloud->points[i - count].y - laserCloud->points[i].y;
        float diffZ2 = laserCloud->points[i - count].z - laserCloud->points[i].z;
        diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
      }

      float depth_right = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                               laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                               laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
      float depth_left = sqrt(laserCloud->points[i - 1].x * laserCloud->points[i - 1].x +
                              laserCloud->points[i - 1].y * laserCloud->points[i - 1].y +
                              laserCloud->points[i - 1].z * laserCloud->points[i - 1].z);

      // outliers
      if ((diff_right[0] > 0.1 * depth && diff_left[0] > 0.1 * depth))
      {
        CloudFeatureFlag[i] = 250;
        continue;
      }

      // break points
      if (fabs(diff_right[0] - diff_left[0]) > 0.1)
      {
        if (diff_right[0] > diff_left[0])
        {

          Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i - 4].x - laserCloud->points[i].x,
                                                        laserCloud->points[i - 4].y - laserCloud->points[i].y,
                                                        laserCloud->points[i - 4].z - laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                         laserCloud->points[i].y,
                                                         laserCloud->points[i].z);
          double left_surf_dis = surf_vector.norm();
          // calculate the angle between the laser direction and the surface
          double cc = fabs(surf_vector.dot(lidar_vector) / (surf_vector.norm() * lidar_vector.norm()));

          std::vector<pcl::PointXYZI> left_list;
          double min_dis = 10000;
          double max_dis = 0;
          for (int j = 0; j < 4; j++)
          { // TODO: change the plane window size and add thin rod support
            left_list.push_back(laserCloud->points[i - j]);
            Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i - j].x - laserCloud->points[i - j - 1].x,
                                                          laserCloud->points[i - j].y - laserCloud->points[i - j - 1].y,
                                                          laserCloud->points[i - j].z - laserCloud->points[i - j - 1].z);

            if (j == 3)
              break;
            double temp_dis = temp_vector.norm();
            if (temp_dis < min_dis)
              min_dis = temp_dis;
            if (temp_dis > max_dis)
              max_dis = temp_dis;
          }
          bool left_is_plane = planeJudge(left_list, 100);

          if (left_is_plane && (max_dis < 2 * min_dis) && left_surf_dis < 0.05 * depth && cc < 0.8)
          {
            if (depth_right > depth_left)
              CloudFeatureFlag[i] = 100;
            else if (depth_right == 0)
              CloudFeatureFlag[i] = 100;
          }
        }
        else
        {

          Eigen::Vector3d surf_vector = Eigen::Vector3d(laserCloud->points[i + 4].x - laserCloud->points[i].x,
                                                        laserCloud->points[i + 4].y - laserCloud->points[i].y,
                                                        laserCloud->points[i + 4].z - laserCloud->points[i].z);
          Eigen::Vector3d lidar_vector = Eigen::Vector3d(laserCloud->points[i].x,
                                                         laserCloud->points[i].y,
                                                         laserCloud->points[i].z);
          double right_surf_dis = surf_vector.norm();
          // calculate the angle between the laser direction and the surface
          double cc = fabs(surf_vector.dot(lidar_vector) / (surf_vector.norm() * lidar_vector.norm()));

          std::vector<pcl::PointXYZI> right_list;
          double min_dis = 10000;
          double max_dis = 0;
          for (int j = 0; j < 4; j++)
          {
            right_list.push_back(laserCloud->points[i + j]); // bugfix 19: https://github.com/Livox-SDK/livox_mapping/pull/19/files
            Eigen::Vector3d temp_vector = Eigen::Vector3d(laserCloud->points[i + j].x - laserCloud->points[i + j + 1].x,
                                                          laserCloud->points[i + j].y - laserCloud->points[i + j + 1].y,
                                                          laserCloud->points[i + j].z - laserCloud->points[i + j + 1].z);

            if (j == 3)
              break;
            double temp_dis = temp_vector.norm();
            if (temp_dis < min_dis)
              min_dis = temp_dis;
            if (temp_dis > max_dis)
              max_dis = temp_dis;
          }
          bool right_is_plane = planeJudge(right_list, 100);

          if (right_is_plane && (max_dis < 2 * min_dis) && right_surf_dis < 0.05 * depth && cc < 0.8)
          {
            if (depth_right < depth_left)
              CloudFeatureFlag[i] = 100;
            else if (depth_left == 0)
              CloudFeatureFlag[i] = 100;
          }
        }
      }

      // break point select
      if (CloudFeatureFlag[i] == 100)
      {
        std::vector<Eigen::Vector3d> front_norms;
        Eigen::Vector3d norm_front(0, 0, 0);
        Eigen::Vector3d norm_back(0, 0, 0);
        for (int k = 1; k < 4; k++)
        {
          Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i - k].x - laserCloud->points[i].x,
                                                laserCloud->points[i - k].y - laserCloud->points[i].y,
                                                laserCloud->points[i - k].z - laserCloud->points[i].z);
          tmp.normalize();
          front_norms.push_back(tmp);
          norm_front += (k / 6.0) * tmp;
        }
        std::vector<Eigen::Vector3d> back_norms;
        for (int k = 1; k < 4; k++)
        {
          Eigen::Vector3d tmp = Eigen::Vector3d(laserCloud->points[i + k].x - laserCloud->points[i].x,
                                                laserCloud->points[i + k].y - laserCloud->points[i].y,
                                                laserCloud->points[i + k].z - laserCloud->points[i].z);
          tmp.normalize();
          back_norms.push_back(tmp);
          norm_back += (k / 6.0) * tmp;
        }
        double cc = fabs(norm_front.dot(norm_back) / (norm_front.norm() * norm_back.norm()));
        if (cc >= 0.8)
          CloudFeatureFlag[i] = 0;

        continue;
      }
    }

    // push_back feature
    pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> surfPointsFlat;
    for (int i = 0; i < cloudSize; i++)
    {
      if (CloudFeatureFlag[i] == 1)
      {
        surfPointsFlat.push_back(laserCloud->points[i]);
        continue;
      }

      if (CloudFeatureFlag[i] == 100 || CloudFeatureFlag[i] == 150)
        cornerPointsSharp.push_back(laserCloud->points[i]);
    }

    //is this better than pub/sub intra process?
    laserMappingObj->updateCB(std::move(cornerPointsSharp), std::move(surfPointsFlat), std::move(cloudInMsg->header.stamp));

    // original
    // sensor_msgs::msg::PointCloud2 cornerPointsSharpMsg;
    // pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    // cornerPointsSharpMsg.header.stamp = cloudInMsg->header.stamp;
    // cornerPointsSharpMsg.header.frame_id = "/livox_frame";
    // pCloudCorner_pub_->publish(cornerPointsSharpMsg);

    // sensor_msgs::msg::PointCloud2 surfPointsFlat2;
    // pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    // surfPointsFlat2.header.stamp = cloudInMsg->header.stamp;
    // surfPointsFlat2.header.frame_id = "/livox_frame";
    // pCloudSurf_pub_->publish(surfPointsFlat2);
  }

  void ScanRegistration::setLaserMapping(const std::shared_ptr<LaserMapping> lm)
  {
    laserMappingObj = lm;
  }

  bool ScanRegistration::planeJudge(const std::vector<pcl::PointXYZI> &pointList, const int planeThresh)
  {
    int num = pointList.size();
    float cx = 0;
    float cy = 0;
    float cz = 0;
    for (int j = 0; j < num; j++)
    {
      cx += pointList[j].x;
      cy += pointList[j].y;
      cz += pointList[j].z;
    }
    cx /= num;
    cy /= num;
    cz /= num;
    // mean square error
    float a11 = 0;
    float a12 = 0;
    float a13 = 0;
    float a22 = 0;
    float a23 = 0;
    float a33 = 0;
    for (int j = 0; j < num; j++)
    {
      float ax = pointList[j].x - cx;
      float ay = pointList[j].y - cy;
      float az = pointList[j].z - cz;

      a11 += ax * ax;
      a12 += ax * ay;
      a13 += ax * az;
      a22 += ay * ay;
      a23 += ay * az;
      a33 += az * az;
    }
    a11 /= num;
    a12 /= num;
    a13 /= num;
    a22 /= num;
    a23 /= num;
    a33 /= num;

    matA1.at<float>(0, 0) = a11;
    matA1.at<float>(0, 1) = a12;
    matA1.at<float>(0, 2) = a13;
    matA1.at<float>(1, 0) = a12;
    matA1.at<float>(1, 1) = a22;
    matA1.at<float>(1, 2) = a23;
    matA1.at<float>(2, 0) = a13;
    matA1.at<float>(2, 1) = a23;
    matA1.at<float>(2, 2) = a33;

    cv::eigen(matA1, matD1, matV1);
    if (matD1.at<float>(0, 0) > planeThresh * matD1.at<float>(0, 1))
      return true;
    else
      return false;
  }


  LaserMapping::LaserMapping() : Node("laser_mapping_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
    loopFreq_(100),
    laserCloudWidth(21),
    laserCloudHeight(11),
    laserCloudDepth(21),
    laserCloudNum(laserCloudWidth * laserCloudHeight * laserCloudDepth),
    transformTobeMapped{0},
    transformAftMapped{0},
    transformLastMapped{0},
    matA0(10, 3, CV_32F, cv::Scalar::all(0)),
    matB0(10, 1, CV_32F, cv::Scalar::all(-1)),
    matX0(10, 1, CV_32F, cv::Scalar::all(0)),
    matA1(3, 3, CV_32F, cv::Scalar::all(0)),
    matD1(1, 3, CV_32F, cv::Scalar::all(0)),
    matV1(3, 3, CV_32F, cv::Scalar::all(0)),
    matP(6, 6, CV_32F, cv::Scalar::all(0)),
    laserCloudCornerArray(laserCloudNum),
    laserCloudCornerArray2(laserCloudNum),
    laserCloudSurfArray(laserCloudNum),
    laserCloudSurfArray2(laserCloudNum),
    laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudCornerLast_down(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudSurfLast_down(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
    coeffSel(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
    laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
    kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
    kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>())
  {
    lidarFrame_ = this->declare_parameter<std::string>("lidar_frame", "livox_frame");
    odomFrame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    baseFrame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");
    filterCorner_ = this->declare_parameter<double>("filter_parameter_corner", 0.1);
    filterSurf_ = this->declare_parameter<double>("filter_parameter_surf", 0.2);

    odom_msg_.header.frame_id = odomFrame_;
    odom_msg_.child_frame_id = baseFrame_;
    laserCloudCenWidth = 10;
    laserCloudCenHeight = 5;
    laserCloudCenDepth = 10;
    isDegenerate = false;
    downSizeFilterCorner.setLeafSize(filterCorner_, filterCorner_, filterCorner_);
    downSizeFilterSurf.setLeafSize(filterSurf_, filterSurf_, filterSurf_);

    for (int i = 0; i < laserCloudNum; i++)
    {
      laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("livox/odom", 1);
    tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    //tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  LaserMapping::~LaserMapping() {}

  void LaserMapping::updateCB(const pcl::PointCloud<pcl::PointXYZI> corner, const pcl::PointCloud<pcl::PointXYZI> surf, const rclcpp::Time ts)
  {
    laserCloudCornerLast->clear();
    laserCloudCornerLast = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI> >(corner);
    laserCloudSurfLast->clear();
    laserCloudSurfLast = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI> >(surf);
    
    // transformAssociateToMap();

    pcl::PointXYZI pointOnYAxis;
    pointOnYAxis.x = 0.0;
    pointOnYAxis.y = 10.0;
    pointOnYAxis.z = 0.0;

    pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

    int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
    int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
    int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

    if (transformTobeMapped[3] + 25.0 < 0)
      centerCubeI--;
    if (transformTobeMapped[4] + 25.0 < 0)
      centerCubeJ--;
    if (transformTobeMapped[5] + 25.0 < 0)
      centerCubeK--;

    while (centerCubeI < 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = laserCloudWidth - 1;

          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; i >= 1; i--)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          }

          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeI++;
      laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = 0;
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; i < laserCloudWidth - 1; i++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeI--;
      laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int j = laserCloudHeight - 1;
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; j >= 1; j--)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeJ++;
      laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int j = 0;
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; j < laserCloudHeight - 1; j++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeJ--;
      laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = laserCloudDepth - 1;
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; k >= 1; k--)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeK++;
      laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = 0;
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

          for (; k < laserCloudDepth - 1; k++)
          {
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
          }
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeCornerPointer;
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCubeSurfPointer;
          laserCloudCubeCornerPointer->clear();
          laserCloudCubeSurfPointer->clear();
        }
      }

      centerCubeK--;
      laserCloudCenDepth--;
    }

    int laserCloudValidNum = 0;
    int laserCloudSurroundNum = 0;

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
        {
          if (i >= 0 && i < laserCloudWidth &&
              j >= 0 && j < laserCloudHeight &&
              k >= 0 && k < laserCloudDepth)
          {

            float centerX = 50.0 * (i - laserCloudCenWidth);
            float centerY = 50.0 * (j - laserCloudCenHeight);
            float centerZ = 50.0 * (k - laserCloudCenDepth);

            bool isInLaserFOV = false;
            for (int ii = -1; ii <= 1; ii += 2)
            {
              for (int jj = -1; jj <= 1; jj += 2)
              {
                for (int kk = -1; kk <= 1; kk += 2)
                {

                  float cornerX = centerX + 25.0 * ii;
                  float cornerY = centerY + 25.0 * jj;
                  float cornerZ = centerZ + 25.0 * kk;

                  float squaredSide1 = (transformTobeMapped[3] - cornerX) * (transformTobeMapped[3] - cornerX) + (transformTobeMapped[4] - cornerY) * (transformTobeMapped[4] - cornerY) + (transformTobeMapped[5] - cornerZ) * (transformTobeMapped[5] - cornerZ);

                  float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY) + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                  float check1 = 100.0 + squaredSide1 - squaredSide2 - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                  float check2 = 100.0 + squaredSide1 - squaredSide2 + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                  if (check1 < 0 && check2 > 0)
                  {
                    isInLaserFOV = true;
                  }
                }
              }
            }

            if (isInLaserFOV)
            {
              laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
              laserCloudValidNum++;
            }
            laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
            laserCloudSurroundNum++;
          }
        }
      }
    }

    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();

    for (int i = 0; i < laserCloudValidNum; i++)
    {
      *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
      *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
    }
    int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
    int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

    laserCloudCornerLast_down->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLast_down);

    laserCloudSurfLast_down->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLast_down);

    if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
    {
      // if (laserCloudSurfFromMapNum > 100) {
      kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
      kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

      for (int iterCount = 0; iterCount < 20; iterCount++)
      {
        laserCloudOri->clear();
        coeffSel->clear();
        for (uint64_t i = 0; i < laserCloudCornerLast->points.size(); i++)
        {
          pointOri = laserCloudCornerLast->points[i];

          pointAssociateToMap(&pointOri, &pointSel);
          // find the closest 5 points
          kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

          if (pointSearchSqDis[4] < 1.5)
          {
            float cx = 0;
            float cy = 0;
            float cz = 0;
            for (int j = 0; j < 5; j++)
            {
              cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
              cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
              cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
            }
            cx /= 5;
            cy /= 5;
            cz /= 5;
            // mean square error
            float a11 = 0;
            float a12 = 0;
            float a13 = 0;
            float a22 = 0;
            float a23 = 0;
            float a33 = 0;
            for (int j = 0; j < 5; j++)
            {
              float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
              float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
              float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

              a11 += ax * ax;
              a12 += ax * ay;
              a13 += ax * az;
              a22 += ay * ay;
              a23 += ay * az;
              a33 += az * az;
            }
            a11 /= 5;
            a12 /= 5;
            a13 /= 5;
            a22 /= 5;
            a23 /= 5;
            a33 /= 5;

            matA1.at<float>(0, 0) = a11;
            matA1.at<float>(0, 1) = a12;
            matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12;
            matA1.at<float>(1, 1) = a22;
            matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13;
            matA1.at<float>(2, 1) = a23;
            matA1.at<float>(2, 2) = a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
            {

              float x0 = pointSel.x;
              float y0 = pointSel.y;
              float z0 = pointSel.z;
              float x1 = cx + 0.1 * matV1.at<float>(0, 0);
              float y1 = cy + 0.1 * matV1.at<float>(0, 1);
              float z1 = cz + 0.1 * matV1.at<float>(0, 2);
              float x2 = cx - 0.1 * matV1.at<float>(0, 0);
              float y2 = cy - 0.1 * matV1.at<float>(0, 1);
              float z2 = cz - 0.1 * matV1.at<float>(0, 2);

              // OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 - z2)，AB = （x1 - x2, y1 - y2, z1 - z2）
              // cross:
              //|  i      j      k  |
              //|x0-x1  y0-y1  z0-z1|
              //|x0-x2  y0-y2  z0-z2|
              float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

              float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

              float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

              float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

              float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

              float ld2 = a012 / l12;
              // if(fabs(ld2) > 1) continue;

              float s = 1 - 0.9 * fabs(ld2);

              coeff.x = s * la;
              coeff.y = s * lb;
              coeff.z = s * lc;
              coeff.intensity = s * ld2;

              if (s > 0.1)
              {
                laserCloudOri->push_back(pointOri);
                coeffSel->push_back(coeff);
              }
            }
          }
        }
        
        for (uint64_t i = 0; i < laserCloudSurfLast_down->points.size(); i++)
        {
          pointOri = laserCloudSurfLast_down->points[i];

          pointAssociateToMap(&pointOri, &pointSel);
          kdtreeSurfFromMap->nearestKSearch(pointSel, 8, pointSearchInd, pointSearchSqDis);

          if (pointSearchSqDis[7] < 5.0)
          {

            for (int j = 0; j < 8; j++)
            {
              matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
              matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
              matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            }
            // matA0*matX0=matB0
            // AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
            //(X,Y,Z)<=>mat_a0
            // A/D, B/D, C/D <=> mat_x0

            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR); // TODO

            float pa = matX0.at<float>(0, 0);
            float pb = matX0.at<float>(1, 0);
            float pc = matX0.at<float>(2, 0);
            float pd = 1;

            // ps is the norm of the plane normal vector
            // pd is the distance from point to plane
            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 8; j++)
            {
              if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
              {
                planeValid = false;
                break;
              }
            }

            if (planeValid)
            {
              // loss fuction
              float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

              // if(fabs(pd2) > 0.1) continue;

              float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

              coeff.x = s * pa;
              coeff.y = s * pb;
              coeff.z = s * pc;
              coeff.intensity = s * pd2;

              if (s > 0.1)
              {
                laserCloudOri->push_back(pointOri);
                coeffSel->push_back(coeff);
              }
            }
          }
        }
        
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        int laserCloudSelNum = laserCloudOri->points.size();
        if (laserCloudSelNum < 50)
        {
          continue;
        }

        //|c1c3+s1s2s3 c3s1s2-c1s3 c2s1|
        //|   c2s3        c2c3      -s2|
        //|c1s2s3-c3s1 c1c3s2+s1s3 c1c2|
        // AT*A*x = AT*b
        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        float debug_distance = 0;
        for (int i = 0; i < laserCloudSelNum; i++)
        {
          pointOri = laserCloudOri->points[i];
          coeff = coeffSel->points[i];

          float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

          float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

          float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

          matA.at<float>(i, 0) = arx;
          matA.at<float>(i, 1) = ary;
          matA.at<float>(i, 2) = arz;
          // TODO: the partial derivative
          matA.at<float>(i, 3) = coeff.x;
          matA.at<float>(i, 4) = coeff.y;
          matA.at<float>(i, 5) = coeff.z;
          matB.at<float>(i, 0) = -coeff.intensity;

          debug_distance += fabs(coeff.intensity);
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        // Deterioration judgment
        if (iterCount == 0)
        {
          cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

          cv::eigen(matAtA, matE, matV);
          matV.copyTo(matV2);

          isDegenerate = false;
          float eignThre[6] = {1, 1, 1, 1, 1, 1};
          for (int i = 5; i >= 0; i--)
          {
            if (matE.at<float>(0, i) < eignThre[i])
            {
              for (int j = 0; j < 6; j++)
              {
                matV2.at<float>(i, j) = 0;
              }
              isDegenerate = true;
            }
            else
            {
              break;
            }
          }
          matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
          cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
          matX.copyTo(matX2);
          matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
            pow(matX.at<float>(0, 0)* 180.0 / M_PI, 2) +
            pow(matX.at<float>(1, 0)* 180.0 / M_PI, 2) +
            pow(matX.at<float>(2, 0)* 180.0 / M_PI, 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05)
        {
          break;
        }
      }

      for (int i = 0; i < 6; i++) {
        transformLastMapped[i] = transformAftMapped[i];
        transformAftMapped[i] = transformTobeMapped[i];
      }
    }

    for (uint64_t i = 0; i < laserCloudCornerLast->points.size(); i++)
    {
      pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);

      int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
      int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
      int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

      if (pointSel.x + 25.0 < 0)
        cubeI--;
      if (pointSel.y + 25.0 < 0)
        cubeJ--;
      if (pointSel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth &&
          cubeJ >= 0 && cubeJ < laserCloudHeight &&
          cubeK >= 0 && cubeK < laserCloudDepth)
      {

        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
    }

    for (uint64_t i = 0; i < laserCloudSurfLast_down->points.size(); i++)
    {
      pointAssociateToMap(&laserCloudSurfLast_down->points[i], &pointSel);

      int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
      int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
      int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

      if (pointSel.x + 25.0 < 0)
        cubeI--;
      if (pointSel.y + 25.0 < 0)
        cubeJ--;
      if (pointSel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth &&
          cubeJ >= 0 && cubeJ < laserCloudHeight &&
          cubeK >= 0 && cubeK < laserCloudDepth)
      {
        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
    }

    for (int i = 0; i < laserCloudValidNum; i++)
    {
      int ind = laserCloudValidInd[i];

      laserCloudCornerArray2[ind]->clear();
      downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
      downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

      laserCloudSurfArray2[ind]->clear();
      downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
      downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

      pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
      laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
      laserCloudCornerArray2[ind] = laserCloudTemp;

      laserCloudTemp = laserCloudSurfArray[ind];
      laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
      laserCloudSurfArray2[ind] = laserCloudTemp;
    }

    tf2::Quaternion q;
    q.setRPY( transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

    
    // geometry_msgs::msg::TransformStamped t;
    // try
    // {
    //   t = tf_buffer_->lookupTransform("base_link", "livox_frame", tf2::TimePointZero);

    //   odom_msg_.header.stamp = ts;
    //   odom_msg_.pose.pose.orientation.x = -q.y();//- t.transform.rotation.x;
    //   odom_msg_.pose.pose.orientation.y = -q.z();// - t.transform.rotation.y;
    //   odom_msg_.pose.pose.orientation.z = q.x();// - t.transform.rotation.z;
    //   odom_msg_.pose.pose.orientation.w = q.w();// - t.transform.rotation.w;
    //   odom_msg_.pose.pose.position.x = -transformAftMapped[3];// - t.transform.translation.x;
    //   odom_msg_.pose.pose.position.y = transformAftMapped[4];// - t.transform.translation.y;
    //   odom_msg_.pose.pose.position.z = transformAftMapped[5];// - t.transform.translation.z;
    //   odom_msg_.pose.covariance = {
    //   0.004,    0.0,    0.0,    0.0,    0.0,    0.0,
    //   0.0,    0.004,    0.0,    0.0,    0.0,    0.0,
    //   0.0,    0.0,    0.004,    0.0,    0.0,    0.0,
    //   0.0,    0.0,    0.0,    0.004,    0.0,    0.0,
    //   0.0,    0.0,    0.0,    0.0,    0.004,    0.0,
    //   0.0,    0.0,    0.0,    0.0,    0.0,    0.004};

    //   odom_pub_->publish(odom_msg_);
    // } 
    // catch (const tf2::TransformException & ex) {} 

    odom_msg_.header.stamp = ts;
    odom_msg_.pose.pose.orientation.x = -q.y();
    odom_msg_.pose.pose.orientation.y = -q.z();
    odom_msg_.pose.pose.orientation.z = q.x();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.pose.pose.position.x = -transformAftMapped[3];
    odom_msg_.pose.pose.position.y = transformAftMapped[4];
    odom_msg_.pose.pose.position.z = transformAftMapped[5];
    odom_msg_.pose.covariance = {
      0.004,    0.0,    0.0,    0.0,    0.0,    0.0,
      0.0,    0.004,    0.0,    0.0,    0.0,    0.0,
      0.0,    0.0,    0.004,    0.0,    0.0,    0.0,
      0.0,    0.0,    0.0,    0.004,    0.0,    0.0,
      0.0,    0.0,    0.0,    0.0,    0.004,    0.0,
      0.0,    0.0,    0.0,    0.0,    0.0,    0.004
    };

    odom_pub_->publish(odom_msg_);

    // geometry_msgs::msg::TransformStamped t;
    // t.header.stamp = ts;
    // t.header.frame_id = odomFrame_;
    // t.child_frame_id = lidarFrame_;
    // t.transform.translation.x = odom_msg_.pose.pose.position.x;
    // t.transform.translation.y = odom_msg_.pose.pose.position.y;
    // t.transform.translation.z = odom_msg_.pose.pose.position.z;
    // t.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
    // t.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
    // t.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
    // t.transform.rotation.w = odom_msg_.pose.pose.orientation.w;
    // tfBroadcaster_->sendTransform(t);
  }

  //lidar coordinate sys to world coordinate sys
  void LaserMapping::pointAssociateToMap(pcl::PointXYZI const * const pi, pcl::PointXYZI * const po)
  {
      //rot z（transformTobeMapped[2]）
      float x1 = cos(transformTobeMapped[2]) * pi->x
              - sin(transformTobeMapped[2]) * pi->y;
      float y1 = sin(transformTobeMapped[2]) * pi->x
              + cos(transformTobeMapped[2]) * pi->y;
      float z1 = pi->z;

      //rot x（transformTobeMapped[0]）
      float x2 = x1;
      float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
      float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

      //rot y（transformTobeMapped[1]）then add trans
      po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
              + transformTobeMapped[3];
      po->y = y2 + transformTobeMapped[4];
      po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
              + transformTobeMapped[5];
      po->intensity = pi->intensity;
  }

} // namespace livox_loam
