#include<ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <livox_ros_driver/CustomMsg.h>
#include<cstdlib>

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

class pointTrans
{
  public:
  pointTrans();
  void registerNodeHandle(ros::NodeHandle& _nh);
  void registerPubSub();
  void livoxCallback (const sensor_msgs::PointCloud::ConstPtr & msg);

  private:
  ros::Publisher gazebo_pub;
  ros::Subscriber gazebo_sub;
  ros::NodeHandle nh;
};

pointTrans::pointTrans()
{
  cout << "=====pointcloud->livox_ros_driver::CustomMsg====="<< endl;
}

void pointTrans::registerNodeHandle(ros::NodeHandle & _nh)
{
  nh = _nh;
}

void pointTrans::registerPubSub()
{
  gazebo_pub = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
  gazebo_sub = nh.subscribe("/livox_scan", 10, &pointTrans::livoxCallback, this);
}

void pointTrans::livoxCallback(const sensor_msgs::PointCloud::ConstPtr & msg)
{
  livox_ros_driver::CustomMsg livox_msg;
  uint point_num = msg->points.size();
  // 预留空间
  livox_msg.points.resize(point_num);
	// 基本信息 https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
  livox_msg.lidar_id = 1;// livox_avia
	livox_msg.point_num = point_num;
	livox_msg.header = msg->header;
  livox_msg.timebase = msg->header.stamp.toSec();

	for (uint i = 0; i < point_num; i++)
	{
		livox_msg.points[i].x =  msg->points[i].x;
		livox_msg.points[i].y =  msg->points[i].y;
		livox_msg.points[i].z =  msg->points[i].z;
		// 激光强度根据距离生成，假设材质都一样
    int distance = int(livox_msg.points[i].x * livox_msg.points[i].x
     + livox_msg.points[i].y * livox_msg.points[i].y +  livox_msg.points[i].z *   livox_msg.points[i].z + 1);
		livox_msg.points[i].reflectivity =  uint8_t (255 / distance);
    livox_msg.points[i].offset_time = msg->header.stamp.toSec();
    // 正常点
    livox_msg.points[i].tag = uint8_t (0000000);
    //livox_msg.points[i].line = uint8_t (rand()%3);
    // cout <<  unsigned(livox_msg.points[i].line) <<endl ;
	}
  ros::Rate loop_rate(10);
  gazebo_pub.publish(livox_msg);
  loop_rate.sleep();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pointcloud2livox");
  ros::NodeHandle nh;
  pointTrans p1;
  p1.registerNodeHandle(nh);
  p1.registerPubSub();
  ros::spin();
}


/*
livox_ros_driver::CustomMsg livox_msg ;

void laserCallback(const sensor_msgs::PointCloud::ConstPtr & msg )
{
	uint point_num = msg->points.size();
  // 预留空间
  livox_msg.points.resize(point_num);
	// 基本信息
  
  https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
  
  livox_msg.lidar_id = 1;// livox_avia
	livox_msg.point_num = point_num;
	livox_msg.header = msg->header;
  livox_msg.timebase = msg->header.stamp.toSec();

	for (uint i = 0; i < point_num; i++)
	{
		livox_msg.points[i].x =  msg->points[i].x;
		livox_msg.points[i].y =  msg->points[i].y;
		livox_msg.points[i].z =  msg->points[i].z;
		// 激光强度根据距离生成，假设材质都一样
    int distance = int(livox_msg.points[i].x * livox_msg.points[i].x
     + livox_msg.points[i].y * livox_msg.points[i].y +  livox_msg.points[i].z *   livox_msg.points[i].z + 1);
		livox_msg.points[i].reflectivity =  uint8_t (255 / distance);
    livox_msg.points[i].offset_time = msg->header.stamp.toSec();
    // 正常点
    livox_msg.points[i].tag = uint8_t (0000000);
    //livox_msg.points[i].line = uint8_t (rand()%3);
    // cout <<  unsigned(livox_msg.points[i].line) <<endl ;
	}
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pointcloud2livox");
  ros::NodeHandle nh;
  ros::Subscriber gazebo_sub = nh.subscribe("/livox_scan", 10, &laserCallback);
  ros::Publisher gazebo_pub = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
  ros::Rate loop_rate(10);
  cout << "*********pointcloud->livox_ros_driver::CustomMsg**********"<< endl;
  while (ros::ok())
  {
    ros::spinOnce();
    if (livox_msg.point_num > 0)
    {
      gazebo_pub.publish(livox_msg);
    }
    livox_msg.points.clear();
    loop_rate.sleep();
  }
  return 0;
}
*/