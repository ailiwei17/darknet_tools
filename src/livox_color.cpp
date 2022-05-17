/*计算box的三维坐标*/
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <vector>
#include <string>

#include <opencv2/core/core.hpp> 

#include <fstream>

#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
/*
darknet_ros_msgs/BoundingBox[] bounding_boxes
float64 probabality
int64 xmin
int64 ymin
int64 xmax
int64 ymax
int16 id
string Class
 */

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

class CameraLidarFusion
{
typedef pcl::PointXYZRGB PointT;

private:
    ros::NodeHandle node_handle_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> livox_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_sub_;
    
    // 3*4的内参投影矩阵，3*3的旋转矩阵和3*1的平移矩阵
    cv::Mat matrix_in;
    cv::Mat matrix_out;
    cv::Mat camera_matrix = cv::Mat::eye(3, 3 ,CV_64F);
    cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
    ofstream ofs;
    tf::TransformListener listener;
    // 存储筛选出的点
    // pcl::PointCloud<PointT>::Ptr cloud;
    
    int distance_x;
    int distance_y;
    
public:
    CameraLidarFusion();
    void callback(const sensor_msgs::PointCloud2ConstPtr& input_livox_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& input_axis_msg);
    void getMat(const string path, vector<float>&Mat,const int line_num);
    void getDistance(const string path, vector<float>&Mat);
    ~CameraLidarFusion();
};

CameraLidarFusion::CameraLidarFusion()
{
    livox_sub_.subscribe(node_handle_,"/cloud_registered_reset",10);
    yolo_sub_.subscribe(node_handle_,"/darknet_ros/bounding_boxes",10);
    
    
    /*
    cloud.reset(new pcl::PointCloud<PointT>());
    cloud->is_dense = false;
    cloud->points.reserve(300000);
    */

    //打开文件
    ofs.open("/home/liwei/catkin_workspace/src/darknet_tools/output.txt",ios::app);
    if (!ofs.is_open())
    {
        cout << "存储文件失败" << endl;
        exit(1);
    }
        
    //获得偏移量
    vector<float> deviation;
    getDistance("/home/liwei/catkin_workspace/src/darknet_tools/yaml/deviation.txt", deviation);
    distance_x = deviation[0];
    distance_y = deviation[1];
        
    vector<float> intrinsic;
    vector<float> extrinsic;
    vector<float> distortion;
    
    //获得参数
    cout << "**********相机内参矩阵为*********" << endl;
    getMat("/home/liwei/catkin_workspace/src/darknet_tools/yaml/intrinsic.txt",intrinsic,3);
    cout << "**********相机畸变矩阵为*********" << endl;
    getMat("/home/liwei/catkin_workspace/src/darknet_tools/yaml/distortion.txt",distortion,1);
    cout << "**********相机外参矩阵为*********" << endl;
    getMat("/home/liwei/catkin_workspace/src/darknet_tools/yaml/extrinsic.txt",extrinsic,3);
    double matrix1[3][4] = {{intrinsic[0], intrinsic[1], intrinsic[2],0}, {intrinsic[3], intrinsic[4], intrinsic[5],0}, {intrinsic[6], intrinsic[7], intrinsic[8],1}};
    double matrix2[4][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]},{0,0,0,1}};
    
    matrix_in = cv::Mat(3, 4, CV_64F, matrix1);
    matrix_out = cv::Mat(4, 4, CV_64F, matrix2);
        
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes>MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), livox_sub_, yolo_sub_);
    sync.registerCallback(boost::bind(&CameraLidarFusion::callback, this, _1, _2));
    ros::spin();
}

CameraLidarFusion::~CameraLidarFusion()
{
    ofs.close();
}

void CameraLidarFusion::callback(const sensor_msgs::PointCloud2ConstPtr& input_livox_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& input_position_msg)
{
    ROS_INFO("sucess");
        
    darknet_ros_msgs::BoundingBoxes bounding_boxes_results = *input_position_msg ;
    // 检测到的目标物体个数
    int num = bounding_boxes_results.bounding_boxes.size();
    // input_livox_msg为雷达坐标系
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_livox_msg, *raw_pcl_ptr);	
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    
    // 每个点保存成pcd
    PointT cloud_add;
    
    geometry_msgs::PointStamped point_livox;
    point_livox.header.stamp = (*input_livox_msg).header.stamp;
    point_livox.header.frame_id = "aft_mapped";
    geometry_msgs::PointStamped point_world;
    
    for(int i = 0;i<num;i++)
    {
        // 减去偏移并缩小框，以提高精度
        int x_long = bounding_boxes_results.bounding_boxes[i].xmax - bounding_boxes_results.bounding_boxes[i].xmin;
        int y_long = bounding_boxes_results.bounding_boxes[i].ymax - bounding_boxes_results.bounding_boxes[i].ymin;
        int x_min = (bounding_boxes_results.bounding_boxes[i].xmin-distance_x) + 0.1 * x_long;
        int x_max = (bounding_boxes_results.bounding_boxes[i].xmax-distance_x) - 0.1 * x_long;
        int y_min = (bounding_boxes_results.bounding_boxes[i].ymin-distance_y) + 0.1 * y_long;
        int y_max = (bounding_boxes_results.bounding_boxes[i].ymax-distance_y) - 0.1 * y_long;
        
        
        cout <<"x:"<< " " << x_min << " "<< x_max << endl; 
        cout <<"y:"<< " " << y_min << " "<< y_max << endl; 
        for(int j = 0;j<raw_pcl_ptr->points.size();j++)
        {
            
            X.at<double>(0,0) = raw_pcl_ptr->points[j].x;
            X.at<double>(1,0) = raw_pcl_ptr->points[j].y;
            X.at<double>(2,0) = raw_pcl_ptr->points[j].z;
            X.at<double>(3,0) = 1;
            // 由雷达坐标转换到相机坐标,再转换到像素坐标
            Y = matrix_in * matrix_out * X; 
            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
            
            // 判断在范围内的点
            if(x_min<pt.x&&pt.x<x_max&&y_min<pt.y&&pt.y<y_max)
            {
                point_livox.point.x = raw_pcl_ptr->points[j].x;
                point_livox.point.y = raw_pcl_ptr->points[j].y;
                point_livox.point.z = raw_pcl_ptr->points[j].z;
                try
                {
                    // 去除当前坐标系下的远点，去除地面点
                    if(point_livox.point.x*point_livox.point.x + point_livox.point.y*point_livox.point.y + point_livox.point.z*point_livox.point.z < 25 && point_livox.point.z>0.1 )
                    {
                        // 把筛选出的点转换回世界坐标系
                        listener.transformPoint("world_reset",point_livox,point_world);
                        cloud_add.x = point_world.point.x;
                        cloud_add.y = point_world.point.y;
                        cloud_add.z = point_world.point.z;
                        cloud_add.b = 255; 

                        // cloud->push_back(cloud_add);

                        //PCL的默认距离单位是:m
                        ofs << "[" <<ros::Time::now().toSec() << "]" << "\t" << point_world.point.x << "\t" << point_world.point.y << "\t" << point_world.point.z << endl;
                    }
                }
                 catch(tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    continue;
                }
            }
                
        }
    }
    
    /*
    if(cloud->points.size()>0)
    {
        pcl::io::savePCDFileASCII("/home/liwei/catkin_workspace/src/learning_image_transport/output.pcd", *cloud);
    }
    */
}

void CameraLidarFusion::getMat(const string path, vector<float>&Mat,const int line_num)
{
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    for (int i = 0; i < line_num; i++) 
    {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;
        while(line>>str)
        {
            Mat.push_back(atof(str.c_str()));
            cout << str << "\t";
        }
        cout << endl;
    }
    inFile.close();
}

void CameraLidarFusion::getDistance(const string path, vector<float>&Mat)
{
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    stringstream line(lineStr);
    string str;
    while(line>>str)
    {
        Mat.push_back(atof(str.c_str()));
    }
    inFile.close();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "LivoxColor");
    CameraLidarFusion cameraLidarFusion;
    return 0;
}

