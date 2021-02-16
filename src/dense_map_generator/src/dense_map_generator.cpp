#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZI PointT;
pcl::PointCloud<PointT>::Ptr cloud_full(new pcl::PointCloud<PointT>());
ros::Publisher testPublisher1;

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  ROS_INFO("I heard: [%d]", cloud_msg->header.seq);
  Eigen::Isometry3d pose = odom2isometry(odom_msg);
  Eigen::Matrix4f pose_m = pose.matrix().cast <float>();
  std::cout << pose_m.matrix() << std::endl;
  sensor_msgs::PointCloud2 cloud_msg_2;
  pcl_ros::transformPointCloud(pose_m, *cloud_msg, cloud_msg_2);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(cloud_msg_2, *cloud);
  *cloud_full += *cloud;
  testPublisher1.publish(cloud_msg_2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dense_map_generator");
  ROS_INFO("Node initialized" );
  ros::NodeHandle nh;
  testPublisher1 = nh.advertise<sensor_msgs::PointCloud2>("transformPointCloud", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom_new", 256);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/velodyne_points_orig", 64);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(32), odom_sub, points_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  if ( !ros::ok() )
  {
     pcl::io::savePCDFileBinary("/home/nithilan/catkin_ws/src/dense_map_generator/dense_map.pcd", *cloud_full);
  }
  return 0;
}