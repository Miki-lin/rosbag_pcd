#include "map_generation_node.h"
//This code is to read the .bag files in ROS and decode them into .png and .pcd
//Coordinate system transformation function is optional. 
using namespace cv;

// 翻滚角roll是围绕x轴旋转，俯仰角pitch是围绕y轴的旋转，偏航角yaw是围绕z轴旋转
// x, y, z 为沿x,y, z轴偏移的距离，与雷达坐标系一致
// 一般情况下，默认为0， 根据雷达安装的位置以及录制的rosbag信息来进行修改
// https://blog.csdn.net/LoongEmbedded/article/details/117554825
double trans_roll_  = 0.0;
double trans_pitch_ = 6.2/57.30;
double trans_yaw_   = 0.0;
double trans_tx_    = 0.720;
double trans_ty_    = 0.0;
double trans_tz_    = 1.500;
// Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();

MapGenerationNode::MapGenerationNode():	lidar_index(0), camera_captured(false), it(nh),
										init_camera_time(false), init_lidar_time(false)
{
    //  激光雷达录制的点云话题
	sub_lidar = nh.subscribe("/rslidar_points", 1, &MapGenerationNode::lidarCallback, this);

	ros::param::set("~image_transport", "compressed");
	// 相机录制的视频话题
	sub_camera = it.subscribe("/cameras/front_upper_camera", 1000, &MapGenerationNode::cameraCallback, this);

    // 通过控制循环的频率，从而控制循环内消息或者服务的发布频率
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		// can add some function
    	ros::spinOnce();
    	loop_rate.sleep();
	}
	// ros::spin();
}

void fromROSMsg_DIY(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	// Get the field structure of this point cloud
	int pointBytes = cloud_msg->point_step;
	int offset_x;
	int offset_y;
	int offset_z;
	int offset_int;
	for (int f=0; f<cloud_msg->fields.size(); ++f)
	{
		if (cloud_msg->fields[f].name == "x")
			offset_x = cloud_msg->fields[f].offset;
		if (cloud_msg->fields[f].name == "y")
			offset_y = cloud_msg->fields[f].offset;
		if (cloud_msg->fields[f].name == "z")
			offset_z = cloud_msg->fields[f].offset;
		if (cloud_msg->fields[f].name == "intensity")
			offset_int = cloud_msg->fields[f].offset;
	}

	// populate point cloud object
	for (int p=0; p<cloud_msg->width*cloud_msg->height; ++p)
	{
		pcl::PointXYZI newPoint;

		newPoint.x = *(float*)(&cloud_msg->data[0] + (pointBytes*p) + offset_x);
		newPoint.y = *(float*)(&cloud_msg->data[0] + (pointBytes*p) + offset_y);
		newPoint.z = *(float*)(&cloud_msg->data[0] + (pointBytes*p) + offset_z);
		newPoint.intensity = *(unsigned char*)(&cloud_msg->data[0] + (pointBytes*p) + offset_int);

		cloud->points.push_back(newPoint);
	}
	cloud->height = cloud_msg->height;
	cloud->width = cloud_msg->width;
}


//About Lidar Points Cloud (Read & Trans & Save)
void MapGenerationNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
	if(!init_lidar_time)
	{
		lidar_base_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6;
		init_lidar_time = true;
	}

	long long lidar_delta_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6 - lidar_base_time;
	ROS_INFO("get lidar : %lld ms", lidar_delta_time);

	char s[200];
	// 点云数据保存路径
	sprintf(s, "/home/xin/code/Tools_RosBag2KITTI/catkin_ws/output/pcd/chuang1_%05lld.pcd", lidar_index); 
	++lidar_index;
	ROS_INFO("lidar index: %05lld", lidar_index);
	camera_captured = false;

	// for (int f=0; f<lidar->fields.size(); ++f)
	// {
	// 	if (lidar->fields[f].name == "intensity")
	// 		std::cout << "intensity lidar->fields[f].datatype = " << (int)lidar->fields[f].datatype << std::endl;
	// }

	// 点云数据PCD的编码格式： XYZI， 4个维度的点云特征编码
	// pcl::fromROSMsg(*lidar, lidar_cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>); // 点云指针
	fromROSMsg_DIY(lidar, lidar_cloud);

	//Building the transformation matrix
	// Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();
	// transform_matrix_.translation() << trans_tx_, trans_ty_, trans_tz_;
  	// transform_matrix_.rotate (Eigen::AngleAxisf(trans_yaw_, Eigen::Vector3f::UnitZ()));
  	// transform_matrix_.rotate (Eigen::AngleAxisf(trans_pitch_, Eigen::Vector3f::UnitY()));
  	// transform_matrix_.rotate (Eigen::AngleAxisf(trans_roll_, Eigen::Vector3f::UnitX()));

	Eigen::Matrix3f rotationMatrix;
    Eigen::Vector3f eulerAngle(trans_roll_, trans_pitch_,trans_yaw_);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitZ()));
    rotationMatrix = yawAngle * pitchAngle * rollAngle;
    Eigen::Translation3f translation(trans_tx_, trans_ty_, trans_tz_);
    Eigen::Matrix4f transfrom = (translation * rotationMatrix).matrix();

  	//Transformation
	pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  	// pcl::transformPointCloud(*lidar_cloud, *trans_cloud_ptr, transform_matrix_);
	pcl::transformPointCloud(*lidar_cloud, *trans_cloud_ptr, transfrom);
	//Save in .pcd
	pcl::io::savePCDFileASCII (s, *trans_cloud_ptr);
}

//About Visual Image (Read & Save)
void MapGenerationNode::cameraCallback(const sensor_msgs::ImageConstPtr& camera)
{
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		// ROS_INFO("discard camera: %lld ms", camera_delta_time);
		return;
	}

	ROS_INFO("get camera: %lld ms", camera_delta_time);
	char s[200];
	sprintf(s, "/home/xin/code/Tools_RosBag2KITTI/catkin_ws/output/png/%05lld.png", lidar_index-1); 

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	imwrite(s, img_rgb);

	camera_captured = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generation");
    MapGenerationNode mapgeneration;
    return 0;
}
