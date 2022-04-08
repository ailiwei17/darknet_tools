/* /raw_in_img为系统时间
 * /cloud_registered为rosbag时间
 * 此节点目的是统一时间，把激光雷达的时间调整为系统时间,/clock有线程问题
 * 订阅tf，把/cloud_registered转换到激光雷达坐标系下
 */
#include "ros/ros.h"
#include <iostream>
#include "ros/time.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

using namespace std;

sensor_msgs::PointCloud2 livox_results;
bool flag = false; 


void LivoxCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    livox_results = *msg;
    // 监听tf变换会消耗时间，容易让时间戳匹配不上
    livox_results.header.stamp =  msg->header.stamp;
    flag = true;
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LivoxTimeReset");
  ros::NodeHandle nh;
  // 创建tf监听器
  tf::TransformListener listener;
  // 创建tf变换关系
  tf::StampedTransform transform;
  // 创建tf广播器
  tf::TransformBroadcaster br;
  
  ros::Subscriber livox_sub;
  livox_sub = nh.subscribe("/cloud_registered",10,&LivoxCallback);
  ros::Publisher livox_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_reset", 10);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (flag)
    {
        // 将点云从世界坐标系转换到雷达坐标系
        pcl_ros::transformPointCloud("aft_mapped",livox_results,livox_results,listener);
        // 更换tf变换时间
        try
        {
            // 得到雷达坐标系转换到世界坐标系的具体变换关系
            listener.lookupTransform("aft_mapped","world" ,livox_results.header.stamp, transform);
            // livox的时间戳必须在tf变换前获得
            livox_results.header.stamp = ros::Time::now();
            // 发布现在时刻的tf变换
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "aft_mapped","world_reset"));
        }
        catch(tf::TransformException &ex)
        {
            // 很有可能没有捕获到tf变换
            ROS_ERROR("%s",ex.what());
            continue;
        }
        livox_pub.publish(livox_results);
        flag = false;
        // 测试tf变换是否正确
        // pcl_ros::transformPointCloud("world_reset",livox_results,livox_results,listener);
        
    }
    loop_rate.sleep();
  }
  return 0;
}
