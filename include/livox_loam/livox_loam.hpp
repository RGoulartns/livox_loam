#ifndef LIVOX_LOAM__LIVOX_LOAM_HPP_
#define LIVOX_LOAM__LIVOX_LOAM_HPP_

#include "livox_loam/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

namespace livox_loam
{

class LaserMapping : public rclcpp::Node
{

public:
  LaserMapping();

  virtual ~LaserMapping();

  void updateCB(const pcl::PointCloud<pcl::PointXYZI> corner, const pcl::PointCloud<pcl::PointXYZI> surf, const rclcpp::Time ts);

private:
  void pointAssociateToMap(pcl::PointXYZI const * const pi, pcl::PointXYZI * const po);
  void pCloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudInMsg);
  void pCloudCornerCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void pCloudSurfCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  const uint8_t loopFreq_;
  nav_msgs::msg::Odometry odom_msg_;
  std::string lidarFrame_, odomFrame_, baseFrame_;
  double filterCorner_, filterSurf_;

  //original vars
  int laserCloudCenWidth, laserCloudCenHeight, laserCloudCenDepth;
  const int laserCloudWidth,laserCloudHeight, laserCloudDepth;
  const int laserCloudNum;
  int laserCloudValidInd[125];
  int laserCloudSurroundInd[125];
  float transformTobeMapped[6], transformAftMapped[6], transformLastMapped[6];
  bool isDegenerate;
  cv::Mat matA0, matB0, matX0, matA1, matD1, matV1, matP;
  
  //std::vector quick workaround to the original format
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudCornerArray, laserCloudCornerArray2;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudSurfArray, laserCloudSurfArray2;
  
  // corner and surf features
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast, laserCloudSurfLast;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast_down, laserCloudSurfLast_down;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri, coeffSel;
  // corner and surf features in map
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap, laserCloudSurfFromMap;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap, kdtreeSurfFromMap;
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  pcl::PointXYZI pointOri, pointSel, coeff;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner, downSizeFilterSurf;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


class ScanRegistration : public rclcpp::Node
{
public:
  ScanRegistration();

  virtual ~ScanRegistration();

  void setLaserMapping(const std::shared_ptr<LaserMapping> lm);

private:
  void pCloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudInMsg);
  bool planeJudge(const std::vector<pcl::PointXYZI>& pointList, const int planeThresh);

  cv::Mat matA1, matD1, matV1;
  int CloudFeatureFlag[32000];

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pCloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pCloudProcessed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pCloudCorner_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pCloudSurf_pub_;

  //tmp
  std::shared_ptr<LaserMapping> laserMappingObj;
};

}  // namespace livox_loam

#endif  // LIVOX_LOAM__LIVOX_LOAM_HPP_
