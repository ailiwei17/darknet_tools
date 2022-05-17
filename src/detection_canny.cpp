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
using namespace cv;

class DetectionCanny
{
	private:
		ros::NodeHandle nh;
		message_filters::Subscriber<sensor_msgs::Image> camera_sub_;
    	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> detection_sub_;
	 	ofstream ofs;
		string dir;
		string dir_point;
	public:
		DetectionCanny();
		void callback(const sensor_msgs::ImageConstPtr& image_msg,const darknet_ros_msgs::BoundingBoxesConstPtr& detection_msg);
		~DetectionCanny();
		
};

DetectionCanny::DetectionCanny()
{
	camera_sub_.subscribe(nh,"/darknet_ros/detection_image",10);
    detection_sub_.subscribe(nh,"/darknet_ros/bounding_boxes",10);
	// 存储边缘坐标
	ros::param::get("dir",dir);
	dir_point = dir + "output/canny.txt";
	ofs.open(dir_point,ios::app);
    if (!ofs.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }


	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes>CannySyncPolicy;
    message_filters::Synchronizer<CannySyncPolicy> sync(CannySyncPolicy(10), camera_sub_, detection_sub_);
    sync.registerCallback(boost::bind(&DetectionCanny::callback, this, _1, _2));
    ros::spin();
}

DetectionCanny::~DetectionCanny()
{
	ofs.close();
}

void DetectionCanny::callback(const sensor_msgs::ImageConstPtr & image_msg,const darknet_ros_msgs::BoundingBoxesConstPtr&detection_msg)
{
	Mat image_raw;
	ROS_INFO("sucess");
	cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(image_msg,sensor_msgs::image_encodings::BGR8);
    image_raw  = cv_ptr->image;
	int num = detection_msg->bounding_boxes.size();
	//遍历检测框
	for(int i = 0;i<num;i++)
	{
		int x_min =   detection_msg->bounding_boxes[i].xmin+5;
		int x_max =  detection_msg->bounding_boxes[i].xmax-5;
		int y_min =   detection_msg->bounding_boxes[i].ymin+5;
		int y_max =   detection_msg->bounding_boxes[i].ymax-5;
		// cout << x_min << "\t" << x_max << "\t"<< y_min << "\t" << y_max  << endl;
		// cout << image_raw.size[0] << "\'t" <<  image_raw.size[1]  << endl;
		// 逐框裁剪
		Mat cropped_image = image_raw(Range(y_min,y_max), Range(x_min,x_max));
		// 边缘
		Mat canny ;
		//边缘点集合
		vector<vector<Point> > contours;
		//边缘关系
		vector<Vec4i> hierarchy;
		// Canny算子提取
		cvtColor(cropped_image, canny, CV_BGR2GRAY);
        blur(canny, canny, Size(7, 7));
        Canny(canny, canny, 0, 30, 3);
		// 提取边界
		findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		// 遍历边界点
		int index = contours.size();
		for(int j =0; j<index;j++)
		{
			int point_num = contours[j].size();
			for(int k=0;k<point_num;k++)
			{
				ofs << contours[j][k].x << "\t" <<   contours[j][k].y << endl;
			}
		}
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "DetectionCanny");
	DetectionCanny detectioncanny;
	return 0;
}