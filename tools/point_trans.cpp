#include<ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;
/* sensor_msgs/PointCloud消息格式
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point32[] points
  float32 x
  float32 y
  float32 z
sensor_msgs/ChannelFloat32[] channels
  string name
  float32[] values
*/

/*livox_ros_driver/CustomMsg 消息格式
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar
*/

livox_ros_driver::CustomMsg livox_msg ;

void laserCallback(const sensor_msgs::PointCloud::ConstPtr & msg )
{
	uint point_num = msg->points.size();
  // 预留空间
  livox_msg.points.resize(point_num);
	// 基本信息
	livox_msg.point_num = point_num;
	livox_msg.header = msg->header;
	for (uint i = 0; i < point_num; i++)
	{
		livox_msg.points[i].x =  msg->points[i].x;
		livox_msg.points[i].y =  msg->points[i].y;
		livox_msg.points[i].z =  msg->points[i].z;
		// 激光强度根据距离生成，假设材质都一样
    int distance = int(livox_msg.points[i].x * livox_msg.points[i].x\
     + livox_msg.points[i].y * livox_msg.points[i].y +  livox_msg.points[i].z *   livox_msg.points[i].z + 1);
		livox_msg.points[i].reflectivity =  255 / distance ;
	}
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud2livox");
  ros::NodeHandle nh;
  ros::Subscriber gazebo_sub = nh.subscribe("/livox_scan", 10, &laserCallback);
  ros::Publisher gazebo_pub = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    gazebo_pub.publish(livox_msg);
    livox_msg.points.clear();
    loop_rate.sleep();
  }
  return 0;
}
