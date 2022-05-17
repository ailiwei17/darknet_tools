/*
获取仿真环境下的机器人真实运动轨迹，并与SLAM算法结果进行比较
 */
#include <ros/ros.h>
#include <iostream>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <nav_msgs/Odometry.h>

using namespace std;

geometry_msgs::PoseStamped pre_pose_now;
void pathCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pre_pose_now.pose.position.x = msg->pose.pose.position.x;
    pre_pose_now.pose.position.y = msg->pose.pose.position.y;
}

int main(int argc,char**argv)
{
    ros::init(argc, argv, "true_pose_publisher");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(0.8);

    //获取机器人名字
    string robot_name;
    n.param<std::string>("robot_name", robot_name, "robot");
    cout<<robot_name<<endl;
    ofstream real_pose;
    real_pose.open("/home/liwei/catkin_workspace/src/darknet_tools/output/real_pose.txt",ios::app);
    if (!real_pose.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }
    ofstream pre_pose;
    pre_pose.open("/home/liwei/catkin_workspace/src/learning_image_transport/output/pre_pose.txt",ios::app);
    if (!pre_pose.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }
    
    //用于获得真实坐标
    ros::ServiceClient client=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Subscriber odomSub = n.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &pathCallback);  //订阅里程计话题信息


    int seq=0;//发布序号
    while(ros::ok())
    {
        ros::spinOnce();
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = robot_name; //指定要获取的机器人在gazebo中的名字；
        if (client.call(srv))
        {
            geometry_msgs::PoseStamped real_pose_now;
            real_pose_now.header.frame_id="gazebo";
            real_pose_now.header.seq=seq++;
            real_pose_now.header.stamp=ros::Time::now();
            real_pose_now.pose=srv.response.pose;
            real_pose << real_pose_now.pose.position.x << "\t" << real_pose_now.pose.position.y << endl;
            pre_pose << pre_pose_now.pose.position.x << "\t" << pre_pose_now.pose.position.y << endl;
        }
    
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
        }
        loop_rate.sleep();
    }

    return 0;
}
