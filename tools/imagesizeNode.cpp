/*本程序用于测试节点图片大小*/
#include "ros/ros.h"
#include "image_transport/image_transport.h" 
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/time.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings; 


cv::Mat imgCallback;
std_msgs::Header header;


void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
      cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      imgCallback = cv_ptr->image;
      cout << imgCallback.cols << " " << imgCallback.rows << endl;
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert");
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tranport");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub;
  ros::Subscriber image_sub;
  //压缩图片节点
  image_sub = nh.subscribe("/darknet_ros/detection_image",10,ImageCallback);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // 此时调用回调函数
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
