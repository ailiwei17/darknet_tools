#include <ros/ros.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fstream>
#include <string.h>

using namespace std;
using namespace message_filters;

class DetectionCanny
{
	private:
		ros::NodeHandle nh;
		message_filters::Subscriber<sensor_msgs::Image> camera_sub_;
    	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_sub_;
	 	ofstream ofs;
		string dir;
	public:
		DetectionCanny();
		
};

DetectionCanny::DetectionCanny()
{
	camera_sub_.subscribe(nh,"/raw_in_img",10);
    yolo_sub_.subscribe(nh,"/darknet_ros/bounding_boxes",10);
	ros::param::get("dir",dir);
	ofs.open(dir,ios::app);
    if (!ofs.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }
}

int main(int argc, char ** argv)
{
	string dir;
	ofstream ofs;
	ros::param::get("dir",dir);
	ofs.open(dir,ios::app);
    if (!ofs.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }
	return 0;
}