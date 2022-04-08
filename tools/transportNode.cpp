/*sensor_msgs/Image->sensor_msgs/CompressedImage由ros提供
本程序实现sensor_msgs/CompressedImage->sensor_msgs/Image*/
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
/* CompressedImage header
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string format
uint8[] data
*/

void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
      cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      imgCallback = cv_ptr_compressed->image;
      header = msg-> header;
      // cv::imshow("imgCallback",imgCallback);
      // cv::waitKey(1);     
      
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
  //发布图片节点
  image_pub = it.advertise("/camera/image_raw", 10);
  //压缩图片节点
  image_sub = nh.subscribe("/camera/image_color/compressed",10,ImageCallback);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // 此时调用回调函数
    ros::spinOnce();
    // 拷贝图像和头文件
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", imgCallback).toImageMsg();
    if (!imgCallback.empty())
    {
       image_pub.publish(msg); 
    }
       
    // double t = msg->header.stamp.toSec();
    // printf("time:%f\n",t);
    loop_rate.sleep();
  }
  return 0;
}


