#include <cstdio>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <livox_loam/livox_loam.hpp>

int main(int argc, char ** argv)
{
  char ns[255];
  gethostname(ns, 255);
  rclcpp::init(argc, argv);

  auto scanReg = std::make_shared<livox_loam::ScanRegistration>();
  auto laserMapping = std::make_shared<livox_loam::LaserMapping>();

  //tmp
  scanReg.get()->setLaserMapping(laserMapping);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(scanReg);
  executor.add_node(laserMapping);
  executor.spin();

  return 0;
}
