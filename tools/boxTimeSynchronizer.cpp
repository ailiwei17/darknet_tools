/*darknet_ros_msgs::BoundingBoxes有两个时间戳
 * header
 * image_header
 * 这个cpp目的是测试darknet_ros_msgs::BoundingBoxes的时间戳
*/
#include "ros/ros.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <iostream>
#include "ros/time.h"
#include <sensor_msgs/PointCloud2.h>

std_msgs::Header header;
darknet_ros_msgs::BoundingBoxes bounding_boxes_results;
sensor_msgs::PointCloud2 livox_results;

void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
   bounding_boxes_results = *msg;
   double t1 = bounding_boxes_results.image_header.stamp.toSec();
   printf("image_header_time:%f\n",t1);
   //header 与 image_header的时间戳相同，为包的时间戳
   //double t2 = bounding_boxes_results.image_header.stamp.toSec();
   //printf("header_time:%f\n",t2);
}

void LivoxCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
   ///cloud_registered的时间戳也是包的时间戳
   livox_results = *msg;
   double t3 = livox_results.header.stamp.toSec();
   printf("livox_header_time:%f\n",t3);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "boxTimeSynchronizer");
    ros::NodeHandle nh;
    ros::Subscriber yolo_sub, livox_sub;
    yolo_sub = nh.subscribe("/darknet_ros/bounding_boxes",10,&BoundingBoxCallback);
    livox_sub = nh.subscribe("/cloud_registered",10,&LivoxCallback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ROS_INFO("Box--->Time");
        // 此时调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

