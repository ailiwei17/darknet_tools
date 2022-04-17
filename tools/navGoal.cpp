/*
从.txt文件读取坐标
处理后计算得到2D导航点，并发布到导航消息中
*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <fstream>
#include <string>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavGoal
{
	private:
		ifstream ifs;
		pcl::PointCloud<pcl::PointXYZ>::Ptr init_points;
		
	public:
		NavGoal();
		~NavGoal();
		vector<Eigen::Vector4f>  Get2DGoal(const int clusterParam);
};

NavGoal::NavGoal()
{
	//加载点云
	init_points.reset(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointXYZ pt;
	// 保存目标点路径
	string path = "/home/liwei/catkin_workspace/src/learning_image_transport/output.txt";
	ifs.open(path, ios::in);
	if (!ifs.is_open())
	{
		cout << "打开文件失败" << endl;
		exit(1);
	}
	string time;
	double x,y,z;
	while(ifs >> time >> x >> y >> z )
	{
		pt.x = x;
		pt.y = y;
		pt.z = z;
		init_points->push_back(pt);
	}
	cout << "已读取" << init_points->points.size() << "个点" << endl;
}

NavGoal::~NavGoal()
{
	ifs.close();
}

vector<Eigen::Vector4f>  NavGoal::Get2DGoal(const int clusterParam)
{
	// 聚类索引向量
	vector<pcl::PointIndices> clusters;
	if (clusterParam == 0)
	{
		//法线估计
		cout << "正在估计点云法线..." << endl;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;									//创建法线估计对象ne
		pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);	//设置搜索方法
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);		//存放法线
		ne.setSearchMethod(tree);
		ne.setInputCloud(init_points);
		ne.setKSearch(20);
		ne.compute(*normals);

		// 区域生长
		cout << "正在进行区域生长..." << endl;
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;	//创建区域生长分割对象
		rg.setMinClusterSize(50);							//设置满足聚类的最小点数
		rg.setMaxClusterSize(99999999);						//设置满足聚类的最大点数
		rg.setSearchMethod(tree);							//设置搜索方法
		rg.setNumberOfNeighbours(50);						//设置邻域搜索的点数
		rg.setInputCloud(init_points);							//设置输入点云
		rg.setInputNormals(normals);						//设置输入点云的法向量
		rg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);		//设置平滑阈值，弧度，用于提取同一区域的点
		rg.setCurvatureThreshold(1);						//设置曲率阈值，如果两个点的法线偏差很小，则测试其曲率之间的差异。如果该值小于曲率阈值，则该算法将使用新添加的点继续簇的增长
		rg.extract(clusters);								//获取聚类结果，并保存到索引向量中
		cout << "目标个数为"<<clusters.size() << endl;
	}
	else if (clusterParam == 1)
	{
		cout <<"正在进行欧式聚类..." << endl;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (init_points);
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.2); //设置近邻搜索的搜索半径为2cm
		ec.setMinClusterSize (10);//设置一个聚类需要的最少点数目为100
		ec.setMaxClusterSize (25000); //设置一个聚类需要的最大点数目为25000
		ec.setSearchMethod (tree);//设置点云的搜索机制
		ec.setInputCloud (init_points);
		ec.extract (clusters);//从点云中提取聚类，并将点云索引保存在cluster_indices中
		cout << "目标个数为"<<clusters.size() << endl;
	}
	

	//提取聚类
	vector<Eigen::Vector4f> cendtroid_list;
	for(vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it < clusters.end();it++ )
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for(vector<int>::const_iterator pit = it->indices.begin(); pit < it->indices.end(); pit ++)
		{
			points_cluster->points.push_back(init_points->points[*pit]);
		}
		// 计算质心
		Eigen::Vector4f centroid; //(x, y, z , 1)
		pcl::compute3DCentroid(*points_cluster, centroid);
		cendtroid_list.push_back(centroid);
	}
	return cendtroid_list;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");
	ros::NodeHandle nh;
	int clusterParam;
	bool isclusterParam = ros::param::get("cluster", clusterParam);
	if(! isclusterParam)
	{
		// 默认欧式聚类
		clusterParam = 1;
		cout << "没有设置参数，默认欧式聚类" << endl;
	}

	NavGoal ng;
	vector<Eigen::Vector4f> ng_list  = ng.Get2DGoal(clusterParam);
	 MoveBaseClient ac("move_base", true);
	
	 while(!ac.waitForServer(ros::Duration(5.0)))
	 {
    		ROS_INFO("Waiting for the move_base action server to come up");
  	}

	move_base_msgs::MoveBaseGoal goal;
	
	// 依次遍历实现导航
	for (int i=0; i < ng_list.size(); i++)
	{
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
	
		goal.target_pose.pose.position.x = ng_list[i][0];
		goal.target_pose.pose.position.y = ng_list[i][1];
		cout << "x:" << "\t" << goal.target_pose.pose.position.x << "\t" << "y:" << goal.target_pose.pose.position.y << endl;
		goal.target_pose.pose.orientation.w = 1.0;
	
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
	
		ac.waitForResult();
	
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved 1 meter forward");
		else
			ROS_INFO("The base failed to move forward 1 meter for some reason");
	}
  	return 0;
}