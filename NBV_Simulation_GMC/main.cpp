#include <windows.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
typedef unsigned long long pop_t;

using namespace std;

#include "Share_Data.hpp"
#include "View_Space.hpp"
#include "Information.hpp"


//Virtual_Perception_3D.hpp new version
void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world, octomap::ColorOcTree* _ground_truth_model, Share_Data* share_data);

class Perception_3D {
public:
	Share_Data* share_data;
	octomap::ColorOcTree* ground_truth_model;
	int full_voxels;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	Perception_3D(Share_Data* _share_data) {
		share_data = _share_data;
		ground_truth_model = share_data->ground_truth_model;
		full_voxels = 0;
		for (octomap::ColorOcTree::leaf_iterator it = ground_truth_model->begin_leafs(), end = ground_truth_model->end_leafs(); it != end; ++it) {
			full_voxels++;
		}
	}

	~Perception_3D() {
		;
	}

	bool precept(View* now_best_view) {
		double now_time = clock();
		//创建当前成像点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parallel(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_parallel->is_dense = false;
		cloud_parallel->points.resize(full_voxels);
		//获取视点位姿
		Eigen::Matrix4d view_pose_world;
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//检查视点的key
		octomap::OcTreeKey key_origin;
		bool key_origin_have = ground_truth_model->coordToKeyChecked(now_best_view->init_pos(0), now_best_view->init_pos(1), now_best_view->init_pos(2), key_origin);
		if (key_origin_have) {
			octomap::point3d origin = ground_truth_model->keyToCoord(key_origin);
			//遍历每个体素
			octomap::point3d* end = new octomap::point3d[full_voxels];
			octomap::ColorOcTree::leaf_iterator it = ground_truth_model->begin_leafs();
			for (int i = 0; i < full_voxels; i++) {
				end[i] = it.getCoordinate();
				it++;
			}
			//ground_truth_model->write(share_data->save_path + "/test_camrea.ot");
			thread** precept_process = new thread * [full_voxels];
			for (int i = 0; i < full_voxels; i++) {
				precept_process[i] = new thread(precept_thread_process, i, cloud_parallel, &origin, &end[i], &view_pose_world, ground_truth_model, share_data);
			}
			for (int i = 0; i < full_voxels; i++)
				(*precept_process[i]).join();
			delete[] end;
			for (int i = 0; i < full_voxels; i++)
				precept_process[i]->~thread();
			delete[] precept_process;
		}
		else {
			cout << "View out of map.check." << endl;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_table(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud = temp;
		cloud->is_dense = false;
		//no_table->is_dense = false;
		cloud->points.resize(full_voxels);
		//no_table->points.resize(full_voxels);
		auto ptr = cloud->points.begin();
		//auto pt = no_table->points.begin();
		int vaild_point = 0;
		//int table_point = 0;
		auto p = cloud_parallel->points.begin();
		for (int i = 0; i < cloud_parallel->points.size(); i++, p++)
		{
			if ((*p).x == 0 && (*p).y == 0 && (*p).z == 0) continue;
			(*ptr).x = (*p).x;
			(*ptr).y = (*p).y;
			(*ptr).z = (*p).z;
			(*ptr).b = (*p).b;
			(*ptr).g = (*p).g;
			(*ptr).r = (*p).r;
			vaild_point++;
			ptr++;
		}
		cloud->width = vaild_point;
		//no_table->width = table_point;
		cloud->height = 1;
		//no_table->height = 1;
		cloud->points.resize(vaild_point);
		//no_table->points.resize(table_point);
		//记录当前采集点云
		share_data->vaild_clouds++;
		share_data->clouds.push_back(cloud);
		//旋转至世界坐标系
		//*share_data->cloud_final += *no_table;
		*share_data->cloud_final += *cloud;
		//cout << "virtual cloud get with executed time " << clock() - now_time << " ms." << endl;
		if (share_data->show) { //显示成像点云
			pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
			viewer1->setBackgroundColor(0, 0, 0);
			viewer1->addCoordinateSystem(0.1);
			viewer1->initCameraParameters();
			viewer1->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			while (!viewer1->wasStopped())
			{
				viewer1->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}

		cloud_parallel->~PointCloud();
		//no_table->~PointCloud();
		cout << "Virtual cloud getted with time " << clock() - now_time << " ms." << endl;
		return true;
	}
};

void precept_thread_process(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, octomap::point3d* _end, Eigen::Matrix4d* _view_pose_world, octomap::ColorOcTree* _ground_truth_model, Share_Data* share_data) {
	//num++;
	octomap::point3d origin = *_origin;
	Eigen::Matrix4d view_pose_world = *_view_pose_world;
	octomap::ColorOcTree* ground_truth_model = _ground_truth_model;
	pcl::PointXYZRGB point;
	point.x = 0; point.y = 0; point.z = 0;
	//投影检测是否在成像范围内
	Eigen::Vector4d end_3d(_end->x(), _end->y(), _end->z(), 1);
	Eigen::Vector4d vertex = view_pose_world.inverse() * end_3d;
	float point_3d[3] = { vertex(0), vertex(1),vertex(2) };
	float pixel[2];
	rs2_project_point_to_pixel(pixel, &share_data->color_intrinsics, point_3d);
	if (pixel[0] < 0 || pixel[0]>share_data->color_intrinsics.width || pixel[1] < 0 || pixel[1]>share_data->color_intrinsics.height) {
		cloud->points[i] = point;
		return;
	}
	//反向投影找到终点
	octomap::point3d end = project_pixel_to_ray_end(pixel[0], pixel[1], share_data->color_intrinsics, view_pose_world, 1.0);
	octomap::OcTreeKey key_end;
	octomap::point3d direction = end - origin;
	octomap::point3d end_point;
	//越过未知区域，找到终点
	bool found_end_point = ground_truth_model->castRay(origin, direction, end_point, true, 6.0 * share_data->predicted_size);
	if (!found_end_point) {//未找到终点，无观测数据
		cloud->points[i] = point;
		return;
	}
	if (end_point == origin) {
		cout << "view in the object. check!" << endl;
		cloud->points[i] = point;
		return;
	}
	//检查一下末端是否在地图限制范围内
	bool key_end_have = ground_truth_model->coordToKeyChecked(end_point, key_end);
	if (key_end_have) {
		octomap::ColorOcTreeNode* node = ground_truth_model->search(key_end);
		if (node != NULL) {
			octomap::ColorOcTreeNode::Color color = node->getColor();
			point.x = end_point.x();
			point.y = end_point.y();
			point.z = end_point.z();
			point.b = color.b;
			point.g = color.g;
			point.r = color.r;
		}
	}
	cloud->points[i] = point;
}


/*
//Virtual_Perception_3D.hpp old version
void precept_thread_process(int x, int y, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, Eigen::Matrix4d* _view_pose_world, Share_Data* share_data);

class Perception_3D {
public:
	Share_Data* share_data;
	octomap::ColorOcTree* ground_truth_model;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	int iterations;

	Perception_3D(Share_Data* _share_data) {
		share_data = _share_data;
		ground_truth_model = share_data->ground_truth_model;
		iterations = 0;
	}

	~Perception_3D() {
		;
	}

	bool precept(View* now_best_view) {
		double now_time = clock();
		//创建当前成像点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parallel(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_parallel->is_dense = false;
		cloud_parallel->points.resize(share_data->color_intrinsics.width * share_data->color_intrinsics.height);
		//获取视点位姿
		Eigen::Matrix4d view_pose_world;
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//检查视点的key
		octomap::OcTreeKey key_origin;
		bool key_origin_have = ground_truth_model->coordToKeyChecked(now_best_view->init_pos(0), now_best_view->init_pos(1), now_best_view->init_pos(2), key_origin);
		if (key_origin_have) {
			octomap::point3d origin = ground_truth_model->keyToCoord(key_origin);
			//遍历像平面
			thread** precept_process = new thread * [share_data->color_intrinsics.width * share_data->color_intrinsics.height];
			for (int x = 0; x < share_data->color_intrinsics.width; x++)
				for (int y = 0; y < share_data->color_intrinsics.height; y++) {
					int i = x * share_data->color_intrinsics.height + y;
					precept_process[i] = new thread(precept_thread_process, x, y, cloud_parallel, &origin, &view_pose_world, share_data);
				}
			for (int x = 0; x < share_data->color_intrinsics.width; x++)
				for (int y = 0; y < share_data->color_intrinsics.height; y++) {
					int i = x * share_data->color_intrinsics.height + y;
					(*precept_process[i]).join();
				}
			//释放内存
			for (int x = 0; x < share_data->color_intrinsics.width; x++)
				for (int y = 0; y < share_data->color_intrinsics.height; y++) {
					int i = x * share_data->color_intrinsics.height + y;
					precept_process[i]->~thread();
				}
			delete precept_process;
		}
		else {
			cout << "View out of map.check." << endl;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud = temp;
		cloud->is_dense = false;
		cloud->points.resize(share_data->color_intrinsics.width * share_data->color_intrinsics.height);
		auto ptr = cloud->points.begin();
		int vaild_point = 0;
		auto p = cloud_parallel->points.begin();
		for (int i = 0; i < cloud_parallel->points.size(); i++, p++)
		{
			if ((*p).x == 0 && (*p).y == 0 && (*p).z == 0) continue;
			(*ptr).x = (*p).x;
			(*ptr).y = (*p).y;
			(*ptr).z = (*p).z;
			(*ptr).b = (*p).b;
			(*ptr).g = (*p).g;
			(*ptr).r = (*p).r;
			vaild_point++;
			ptr++;
		}
		cloud->width = vaild_point;
		cloud->height = 1;
		cloud->points.resize(vaild_point);
		//记录当前采集点云
		share_data->vaild_clouds++;
		share_data->clouds.push_back(cloud);
		//旋转至世界坐标系
		*share_data->cloud_final += *cloud;
		cout << "virtual cloud get with executed time " << clock() - now_time << " ms." << endl;
		if (share_data->show) { //显示成像点云
			pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Camera"));
			viewer1->setBackgroundColor(0, 0, 0);
			viewer1->addCoordinateSystem(0.1);
			viewer1->initCameraParameters();
			viewer1->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer1->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			while (!viewer1->wasStopped())
			{
				viewer1->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}
		iterations++;
		cloud_parallel->~PointCloud();
		return true;
	}
};

void precept_thread_process(int x, int y, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, octomap::point3d* _origin, Eigen::Matrix4d* _view_pose_world, Share_Data* share_data) {
	//num++;
	octomap::point3d origin = *_origin;
	Eigen::Matrix4d view_pose_world = *_view_pose_world;
	cv::Point2f pixel(x, y);
	//反向投影找到终点
	octomap::point3d end = project_pixel_to_ray_end(x, y, share_data->color_intrinsics, view_pose_world, 1.0);
	//显示一下
	octomap::OcTreeKey key_end;
	octomap::point3d direction = end - origin;
	octomap::point3d end_point;
	pcl::PointXYZRGB point;
	point.x = 0;
	point.y = 0;
	point.z = 0;
	point.b = 0;
	point.g = 0;
	point.r = 0;
	//越过未知区域，找到终点
	bool found_end_point = share_data->ground_truth_model->castRay(origin, direction, end_point, true, 6.0 * share_data->predicted_size);
	if (!found_end_point) {//未找到终点，无观测数据
		cloud->points[x * share_data->color_intrinsics.height + y] = point;
		return;
	}
	if (end_point == origin) {
		cout << "view in the object. check!" << endl;
		cloud->points[x * share_data->color_intrinsics.height + y] = point;
		return;
	}
	//检查一下末端是否在地图限制范围内
	bool key_end_have = share_data->ground_truth_model->coordToKeyChecked(end_point, key_end);
	if (key_end_have) {
		octomap::ColorOcTreeNode* node = share_data->ground_truth_model->search(key_end);
		if (node != NULL) {
			octomap::ColorOcTreeNode::Color color = node->getColor();
			point.x = end_point.x();
			point.y = end_point.y();
			point.z = end_point.z();
			point.b = color.b;
			point.g = color.g;
			point.r = color.r;
		}
	}
	cloud->points[x * share_data->color_intrinsics.height + y] = point;
}
*/

#define Over 0
#define WaitData 1
#define WaitViewSpace 2
#define WaitInformation 3
#define WaitMoving 4

//NVB_Planner.hpp
void save_cloud_mid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string name);
void create_view_space(View_Space** now_view_space, View* now_best_view, Share_Data* share_data, int iterations);
void create_views_information(Views_Information** now_views_infromation, View* now_best_view, View_Space* now_view_space, Share_Data* share_data, int iterations);
void move_robot(View* now_best_view, View_Space* now_view_space, Share_Data* share_data);
void show_cloud(pcl::visualization::PCLVisualizer::Ptr viewer);

class NBV_Planner
{
public:
	atomic<int> status;
	int iterations;
	Perception_3D* percept;
	Voxel_Information* voxel_information;
	View_Space* now_view_space;
	Views_Information* now_views_infromation;
	View* now_best_view;
	Share_Data* share_data;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	~NBV_Planner() {
		delete percept;
		delete now_best_view;
		delete voxel_information;
		delete now_view_space;
		if(share_data->method_of_IG != NBVNET && share_data->method_of_IG != PCNBV) delete now_views_infromation;
	}

	double check_size(double predicted_size, Eigen::Vector3d object_center_world, vector<Eigen::Vector3d>& points) {
		int vaild_points = 0;
		for (auto& ptr : points) {
			if (ptr(0) < object_center_world(0) - predicted_size || ptr(0) > object_center_world(0) + predicted_size) continue;
			if (ptr(1) < object_center_world(1) - predicted_size || ptr(1) > object_center_world(1) + predicted_size) continue;
			if (ptr(2) < object_center_world(2) - predicted_size || ptr(2) > object_center_world(2) + predicted_size) continue;
			vaild_points++;
		}
		return (double)vaild_points / (double)points.size();
	}

	NBV_Planner(Share_Data* _share_data, int _status = WaitData) {
		share_data = _share_data;
		iterations = 0;
		share_data->iterations = 0;
		status = _status;
		share_data->now_view_space_processed = false;
		share_data->now_views_infromation_processed = false;
		share_data->move_on = false;
		voxel_information = new Voxel_Information(share_data->p_unknown_lower_bound, share_data->p_unknown_upper_bound);
		voxel_information->init_mutex_views(share_data->num_of_views);
		//初始化GT
		//share_data->access_directory(share_data->save_path);
		//GT cloud
		share_data->cloud_ground_truth->is_dense = false;
		share_data->cloud_ground_truth->points.resize(share_data->cloud_pcd->points.size());
		share_data->cloud_ground_truth->width = share_data->cloud_pcd->points.size();
		share_data->cloud_ground_truth->height = 1;
		auto ptr = share_data->cloud_ground_truth->points.begin();
		auto p = share_data->cloud_pcd->points.begin();
		float unit = 1.0;
		for (auto& ptr : share_data->cloud_pcd->points) {
			if (fabs(ptr.x) >= 10 || fabs(ptr.y) >= 10 || fabs(ptr.z) >= 10) {
				unit = 0.001;
				cout << "change unit from <mm> to <m>." << endl;
				break;
			}
		}
		//检查物体大小，统一缩放为0.15m左右
		vector<Eigen::Vector3d> points;
		for (auto& ptr : share_data->cloud_pcd->points) {
			Eigen::Vector3d pt(ptr.x * unit, ptr.y * unit, ptr.z * unit);
			points.push_back(pt);
		}
		Eigen::Vector3d object_center_world = Eigen::Vector3d(0, 0, 0);
		//计算点云质心
		for (auto& ptr : points) {
			object_center_world(0) += ptr(0);
			object_center_world(1) += ptr(1);
			object_center_world(2) += ptr(2);
		}
		object_center_world(0) /= points.size();
		object_center_world(1) /= points.size();
		object_center_world(2) /= points.size();
		//二分查找BBX半径，以BBX内点的个数比率达到0.90-0.95为终止条件
		double l = 0, r = 0, mid;
		for (auto& ptr : points) {
			r = max(r, (object_center_world - ptr).norm());
		}
		mid = (l + r) / 2;
		double precent = check_size(mid, object_center_world, points);
		double pre_precent = precent;
		while (precent > 0.95 || precent < 1.0) {
			if (precent > 0.95) {
				r = mid;
			}
			else if (precent < 1.0) {
				l = mid;
			}
			mid = (l + r) / 2;
			precent = check_size(mid, object_center_world, points);
			if (fabs(pre_precent - precent) < 0.001) break;
			pre_precent = precent;
		}
		double predicted_size = 1.2 * mid;
		float scale = 1.0;
		if (share_data->resize_scale >= 0) {
			scale = share_data->resize_scale / predicted_size;
			cout << "object " << share_data->name_of_pcd<<" large. change scale "<< predicted_size <<" to about " << share_data->resize_scale <<" m." << endl;
		}
		//转换点云
		for (int i = 0; i < share_data->cloud_pcd->points.size(); i++, p++)
		{
			(*ptr).x = (*p).x * scale * unit;
			(*ptr).y = (*p).y * scale * unit;
			(*ptr).z = (*p).z * scale * unit;
			(*ptr).b = 168;
			(*ptr).g = 168;
			(*ptr).r = 168;
			//GT插入点云
			octomap::OcTreeKey key;  bool key_have = share_data->ground_truth_model->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key);
			if (key_have) {
				octomap::ColorOcTreeNode* voxel = share_data->ground_truth_model->search(key);
				if (voxel == NULL) {
					share_data->ground_truth_model->setNodeValue(key, share_data->ground_truth_model->getProbHitLog(), true);
					share_data->ground_truth_model->integrateNodeColor(key, (*ptr).r, (*ptr).g, (*ptr).b);
				}
			}
			//GT_sample插入点云
			octomap::OcTreeKey key_sp;  bool key_have_sp = share_data->GT_sample->coordToKeyChecked(octomap::point3d((*ptr).x, (*ptr).y, (*ptr).z), key_sp);
			if (key_have_sp) {
				octomap::ColorOcTreeNode* voxel_sp = share_data->GT_sample->search(key_sp);
				if (voxel_sp == NULL) {
					share_data->GT_sample->setNodeValue(key_sp, share_data->GT_sample->getProbHitLog(), true);
					share_data->GT_sample->integrateNodeColor(key_sp, (*ptr).r, (*ptr).g, (*ptr).b);
				}
			}
			ptr++;
		}
		share_data->ground_truth_model->updateInnerOccupancy();
		//share_data->ground_truth_model->write(share_data->save_path + "/GT.ot");
		//GT_sample_voxels
		share_data->GT_sample->updateInnerOccupancy();
		//share_data->GT_sample->write(share_data->save_path + "/GT_sample.ot");
		share_data->init_voxels = 0;
		for (octomap::ColorOcTree::leaf_iterator it = share_data->GT_sample->begin_leafs(), end = share_data->GT_sample->end_leafs(); it != end; ++it){
			share_data->init_voxels++;
		}
		cout << "Map_GT_sample has voxels " << share_data->init_voxels << endl;
		//ofstream fout(share_data->save_path + "/GT_sample_voxels.txt");
		//fout << share_data->init_voxels << endl;
		//在GT中统计总个数
		share_data->cloud_points_number = 0;
		for (octomap::ColorOcTree::leaf_iterator it = share_data->ground_truth_model->begin_leafs(), end = share_data->ground_truth_model->end_leafs(); it != end; ++it) {
			share_data->cloud_points_number++;
		}
		cout << "Map_GT has voxels " << share_data->cloud_points_number << endl;

		//初始化viewspace
		int first_view_id = share_data->first_view_id;
		now_view_space = new View_Space(iterations, share_data, voxel_information, share_data->cloud_ground_truth, first_view_id);
		//设置初始视点为统一的位置
		now_view_space->views[first_view_id].vis++;
		now_best_view = new View(now_view_space->views[first_view_id]);
		now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
		Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();
		//运动代价：视点id，当前代价，总体代价
		share_data->movement_cost = 0;
		//share_data->access_directory(share_data->save_path + "/movement");
		//ofstream fout_move(share_data->save_path + "/movement/path" + to_string(-1) + ".txt");
		//fout_move << 0 << '\t' << 0.0 << '\t' << 0.0 << endl;
		//相机类初始化
		percept = new Perception_3D(share_data);
		if (share_data->show) { //显示BBX、相机位置、GT
			pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Iteration"));
			viewer->setBackgroundColor(255, 255, 255);
			//viewer->addCoordinateSystem(0.1);
			viewer->initCameraParameters();
			//第一帧相机位置
			Eigen::Vector4d X(0.05, 0, 0, 1);
			Eigen::Vector4d Y(0, 0.05, 0, 1);
			Eigen::Vector4d Z(0, 0, 0.05, 1);
			Eigen::Vector4d O(0, 0, 0, 1);
			X = view_pose_world * X;
			Y = view_pose_world * Y;
			Z = view_pose_world * Z;
			O = view_pose_world * O;
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(-1));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(-1));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(-1));
			//test_viewspace
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_viewspace(new pcl::PointCloud<pcl::PointXYZRGB>);
			test_viewspace->is_dense = false;
			test_viewspace->points.resize(now_view_space->views.size());
			auto pt = test_viewspace->points.begin();
			for (int i = 0; i < now_view_space->views.size(); i++, pt++) {
				(*pt).x = now_view_space->views[i].init_pos(0);
				(*pt).y = now_view_space->views[i].init_pos(1);
				(*pt).z = now_view_space->views[i].init_pos(2);
				//第一次显示所有点为白色
				(*pt).r = 0, (*pt).g = 0, (*pt).b = 0;
			}
			viewer->addPointCloud<pcl::PointXYZRGB>(test_viewspace, "test_viewspace");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "test_viewspace");
			now_view_space->add_bbx_to_cloud(viewer);
			viewer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_ground_truth, "cloud_ground_truth");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_ground_truth");
			while (!viewer->wasStopped())
			{
				viewer->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}
	}

	int plan() {
		switch (status)
		{
		case Over:
			break;
		case WaitData:
			if (percept->precept(now_best_view)) {
				thread next_view_space(create_view_space, &now_view_space, now_best_view, share_data, iterations);
				next_view_space.detach();
				status = WaitViewSpace;
			}
			break;
		case WaitViewSpace:
			if (share_data->now_view_space_processed) {
				thread next_views_information(create_views_information, &now_views_infromation, now_best_view, now_view_space, share_data, iterations);
				next_views_information.detach();
				status = WaitInformation;
			}
			break;
		case WaitInformation:
			if (share_data->now_views_infromation_processed) {
				if (share_data->method_of_IG == 6) { //NBV-NET
					share_data->access_directory(share_data->nbv_net_path + "/log");
					ifstream ftest;
					do {
						ftest.open(share_data->nbv_net_path + "/log/ready.txt");
					} while (!ftest.is_open());
					ftest.close();
					ifstream fin(share_data->nbv_net_path + "/log/" + share_data->name_of_pcd + "_v" + to_string(share_data->first_view_id) + '_' + to_string(iterations) + ".txt");
					//double x, y, z, a, b, c;
					//fin >> x >> y >> z >> a >> b >> c;
					double x, y, z;
					fin >> x >> y >> z;
					cout << x << " " << y << " " << z <<endl;
					now_best_view->init_pos(0) = x;
					now_best_view->init_pos(1) = y;
					now_best_view->init_pos(2) = z;
					now_best_view->get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
					/*Eigen::Matrix3d rotation;
					rotation = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()) *
						Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ());
					Eigen::Matrix4d R(Eigen::Matrix4d::Identity(4, 4));
					R(0, 0) = rotation(0, 0); R(0, 1) = rotation(0, 1); R(0, 2) = rotation(0, 2); R(0, 3) = x;
					R(1, 0) = rotation(1, 0); R(1, 1) = rotation(1, 1); R(1, 2) = rotation(1, 2); R(1, 3) = y;
					R(2, 0) = rotation(2, 0); R(2, 1) = rotation(2, 1); R(2, 2) = rotation(2, 2); R(2, 3) = z;
					R(3, 0) = 0;			  R(3, 1) = 0;			    R(3, 2) = 0;			  R(3, 3) = 1;
					now_best_view->pose = R;*/
					//运动代价：视点id，当前代价，总体代价
					pair<int, double> local_path = get_local_path(Eigen::Vector3d(share_data->now_camera_pose_world(0, 3), share_data->now_camera_pose_world(1, 3), share_data->now_camera_pose_world(2, 3)).eval(), now_best_view->init_pos.eval(), now_view_space->object_center_world.eval(), now_view_space->predicted_size * sqrt(2)); //包围盒半径是半边长的根号2倍
					if (local_path.first < 0) cout << "local path wrong." << endl;
					now_best_view->robot_cost = local_path.second;
					share_data->movement_cost += now_best_view->robot_cost;
					share_data->access_directory(share_data->save_path + "/movement");
					ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
					fout_move << -1 << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
					this_thread::sleep_for(chrono::seconds(1));
					int removed = remove((share_data->nbv_net_path + "/log/ready.txt").c_str());
					if (removed!=0) cout << "cannot remove ready.txt." << endl;
				}
				else{//搜索算法
					//对视点排序
					sort(now_view_space->views.begin(), now_view_space->views.end(), view_utility_compare);
					/*if (share_data->sum_local_information == 0) {
						cout << "randomly choose a view" << endl;
						srand(clock());
						random_shuffle(now_view_space->views.begin(), now_view_space->views.end());
					}*/
					//informed_viewspace
					if (share_data->show) { //显示BBX与相机位置
						pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Iteration" + to_string(iterations)));
						viewer->setBackgroundColor(0, 0, 0);
						viewer->addCoordinateSystem(0.1);
						viewer->initCameraParameters();
						//test_viewspace
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_viewspace(new pcl::PointCloud<pcl::PointXYZRGB>);
						test_viewspace->is_dense = false;
						test_viewspace->points.resize(now_view_space->views.size());
						auto ptr = test_viewspace->points.begin();
						int needed = 0;
						for (int i = 0; i < now_view_space->views.size(); i++) {
							(*ptr).x = now_view_space->views[i].init_pos(0);
							(*ptr).y = now_view_space->views[i].init_pos(1);
							(*ptr).z = now_view_space->views[i].init_pos(2);
							//访问过的点记录为蓝色
							if (now_view_space->views[i].vis) (*ptr).r = 0, (*ptr).g = 0, (*ptr).b = 255;
							//在网络流内的设置为黄色
							else if (now_view_space->views[i].in_coverage[iterations] && i < now_view_space->views.size() / 10) (*ptr).r = 255, (*ptr).g = 255, (*ptr).b = 0;
							//在网络流内的设置为绿色
							else if (now_view_space->views[i].in_coverage[iterations]) (*ptr).r = 255, (*ptr).g = 0, (*ptr).b = 0;
							//前10%的权重的点设置为蓝绿色
							else if (i < now_view_space->views.size() / 10) (*ptr).r = 0, (*ptr).g = 255, (*ptr).b = 255;
							//其余点不要了
							else continue;
							ptr++;
							needed++;
						}
						test_viewspace->points.resize(needed);
						viewer->addPointCloud<pcl::PointXYZRGB>(test_viewspace, "test_viewspace");
						bool best_have = false;
						for (int i = 0; i < now_view_space->views.size(); i++) {
							if (now_view_space->views[i].vis) {
								now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
								Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse()).eval();
								Eigen::Vector4d X(0.03, 0, 0, 1);
								Eigen::Vector4d Y(0, 0.03, 0, 1);
								Eigen::Vector4d Z(0, 0, 0.03, 1);
								Eigen::Vector4d O(0, 0, 0, 1);
								X = view_pose_world * X;
								Y = view_pose_world * Y;
								Z = view_pose_world * Z;
								O = view_pose_world * O;
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(i));
							}
							else if (!best_have) {
								now_view_space->views[i].get_next_camera_pos(share_data->now_camera_pose_world, share_data->object_center_world);
								Eigen::Matrix4d view_pose_world = (share_data->now_camera_pose_world * now_view_space->views[i].pose.inverse()).eval();
								Eigen::Vector4d X(0.08, 0, 0, 1);
								Eigen::Vector4d Y(0, 0.08, 0, 1);
								Eigen::Vector4d Z(0, 0, 0.08, 1);
								Eigen::Vector4d O(0, 0, 0, 1);
								X = view_pose_world * X;
								Y = view_pose_world * Y;
								Z = view_pose_world * Z;
								O = view_pose_world * O;
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(X(0), X(1), X(2)), 255, 0, 0, "X" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Y(0), Y(1), Y(2)), 0, 255, 0, "Y" + to_string(i));
								viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(O(0), O(1), O(2)), pcl::PointXYZ(Z(0), Z(1), Z(2)), 0, 0, 255, "Z" + to_string(i));
								best_have = true;
							}
						}
						viewer->addPointCloud<pcl::PointXYZRGB>(share_data->cloud_final, "cloud_now_itreation");
						while (!viewer->wasStopped())
						{
							viewer->spinOnce(100);
							boost::this_thread::sleep(boost::posix_time::microseconds(100000));
						}
					}
					double max_utility = -1;
					for (int i = 0; i < now_view_space->views.size(); i++) {
						cout << "checking view " << i << endl;
						if (now_view_space->views[i].vis) continue;
						//if (!now_view_space->views[i].can_move) continue;
						now_best_view = new View(now_view_space->views[i]);
						max_utility = now_best_view->final_utility;
						now_view_space->views[i].vis++;
						now_view_space->views[i].can_move = true;
						cout << "choose the " << i << "th view." << endl;
						//运动代价：视点id，当前代价，总体代价
						share_data->movement_cost += now_best_view->robot_cost;
						share_data->access_directory(share_data->save_path + "/movement");
						ofstream fout_move(share_data->save_path + "/movement/path" + to_string(iterations) + ".txt");
						fout_move << now_best_view->id << '\t' << now_best_view->robot_cost << '\t' << share_data->movement_cost << endl;
						break;
					}
					if (max_utility == -1) {
						cout << "Can't move to any viewport.Stop." << endl;
						status = Over;
						break;
					}
					cout << " next best view pos is (" << now_best_view->init_pos(0) << ", " << now_best_view->init_pos(1) << ", " << now_best_view->init_pos(2) << ")" << endl;
					cout << " next best view final_utility is " << now_best_view->final_utility << endl;
				}
				thread next_moving(move_robot, now_best_view, now_view_space, share_data);
				next_moving.detach();
				status = WaitMoving;
			}
			break;
		case WaitMoving:
			//virtual move
			if (share_data->over) {
				cout << "Progress over.Saving octomap and cloud." << endl;
				status = Over;
				break;
			}
			if (share_data->move_on) {
				iterations++;
				share_data->iterations++;
				share_data->now_view_space_processed = false;
				share_data->now_views_infromation_processed = false;
				share_data->move_on = false;
				status = WaitData;
			}
			break;
		}
		return status;
	}

	string out_status() {
		string status_string;
		switch (status)
		{
		case Over:
			status_string = "Over";
			break;
		case WaitData:
			status_string = "WaitData";
			break;
		case WaitViewSpace:
			status_string = "WaitViewSpace";
			break;
		case WaitInformation:
			status_string = "WaitInformation";
			break;
		case WaitMoving:
			status_string = "WaitMoving";
			break;
		}
		return status_string;
	}
};

atomic<bool> stop = false;		//控制程序结束
Share_Data* share_data;			//共享数据区指针
NBV_Planner* nbv_plan;

void save_cloud_mid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string name) {
	//保存中间的点云的线程，目前不检查是否保存完毕
	share_data->save_cloud_to_disk(cloud, "/clouds", name);
	cout << name << " saved" << endl;
	cloud->~PointCloud();
}

void create_view_space(View_Space** now_view_space, View* now_best_view, Share_Data* share_data, int iterations) {
	//计算关键帧相机位姿
	share_data->now_camera_pose_world = (share_data->now_camera_pose_world * now_best_view->pose.inverse()).eval();;
	//处理viewspace
	(*now_view_space)->update(iterations, share_data, share_data->cloud_final, share_data->clouds[iterations]);
	//保存中间迭代结果
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_mid = *share_data->cloud_final;
	thread save_mid(save_cloud_mid, cloud_mid, "pointcloud" + to_string(iterations));
	save_mid.detach();
	//更新标志位
	share_data->now_view_space_processed = true;
}

void create_views_information(Views_Information** now_views_infromation, View* now_best_view, View_Space* now_view_space, Share_Data* share_data, int iterations) {
	if (share_data->method_of_IG == 6) { //NBV-NET
		//scale
		share_data->access_directory(share_data->nbv_net_path + "/viewspace");
		ofstream fout_vs(share_data->nbv_net_path + "/viewspace/" + share_data->name_of_pcd + ".txt");
		fout_vs << now_view_space->predicted_size * 3.0 << '\n';
		/*double scale_of_object = 0;
		double x1 = now_view_space->object_center_world(0) - now_view_space->predicted_size;
		double x2 = now_view_space->object_center_world(0) + now_view_space->predicted_size;
		double y1 = now_view_space->object_center_world(1) - now_view_space->predicted_size;
		double y2 = now_view_space->object_center_world(1) + now_view_space->predicted_size;
		double z1 = now_view_space->object_center_world(2) - now_view_space->predicted_size;
		double z2 = now_view_space->object_center_world(2) + now_view_space->predicted_size;
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1,y1,z1).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1,y2,z1).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1,y1,z2).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x1,y2,z2).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2,y1,z1).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2,y2,z1).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2,y1,z2).eval()).norm());
		scale_of_object = max(scale_of_object, (Eigen::Vector3d(x2,y2,z2).eval()).norm());
		fout_vs << scale_of_object * 2.0 << '\n';*/
		//octotree
		share_data->access_directory(share_data->nbv_net_path + "/data");
		ofstream fout(share_data->nbv_net_path + "/data/" + share_data->name_of_pcd + "_v" + to_string(share_data->first_view_id) + '_' + to_string(iterations) +".txt");
		int voxel_num = 0;
		for(double x = now_view_space->object_center_world(0) - 16 * share_data->octomap_resolution; x<= now_view_space->object_center_world(0) + 16 * share_data->octomap_resolution; x+= share_data->octomap_resolution)
			for(double y = now_view_space->object_center_world(1) - 16 * share_data->octomap_resolution; y<= now_view_space->object_center_world(1) + 16 * share_data->octomap_resolution; y+= share_data->octomap_resolution)
				for (double z = now_view_space->object_center_world(2) - 16 * share_data->octomap_resolution; z <= now_view_space->object_center_world(2) + 16 * share_data->octomap_resolution; z += share_data->octomap_resolution) 
				{
					octomap::OcTreeKey key = share_data->octo_model->coordToKey(x, y, z);
					bool key_have = share_data->octo_model->coordToKeyChecked(octomap::point3d(x,y,z), key);
					if (key_have) {
						octomap::ColorOcTreeNode* node = share_data->octo_model->search(key);
						if (node != NULL) {
							double occupancy = node->getOccupancy();
							fout << occupancy << '\n';
						}
						else {
							fout << 0.5 << '\n';
							//fout << 0.0 << '\n';
						}
					}
					else {
						fout << 0.5 << '\n';
						//fout << 0.0 << '\n';
					}
					voxel_num++;
				}
		if (voxel_num != 32 * 32 * 32) cout << "Voxel gird num wrong!" << endl;
		/*for (octomap::ColorOcTree::leaf_iterator it = share_data->octo_model->begin_leafs(), end = share_data->octo_model->end_leafs(); it != end; ++it) {
			double x = it.getX(); double y = it.getY(); double z = it.getZ();
			if (x >= now_view_space->object_center_world(0) - now_view_space->predicted_size && x <= now_view_space->object_center_world(0) + now_view_space->predicted_size
				&& y >= now_view_space->object_center_world(1) - now_view_space->predicted_size && y <= now_view_space->object_center_world(1) + now_view_space->predicted_size
				&& z >= now_view_space->object_center_world(2) - now_view_space->predicted_size && z <= now_view_space->object_center_world(2) + now_view_space->predicted_size) {
				double occupancy = (*it).getOccupancy();
				fout << x << ' ' << y << ' ' << z << ' '<< occupancy<<'\n';
			}
		}*/
	}
	else { //搜索方法
		int num_of_cover = 1;
		int num_of_voxel = 0;
		//处理views_informaiton
		if (iterations == 0) (*now_views_infromation) = new Views_Information(share_data, nbv_plan->voxel_information, now_view_space, iterations);
		else (*now_views_infromation)->update(share_data, now_view_space, iterations);
		if (share_data->method_of_IG == GMC) {
			if (share_data->num_of_max_iteration - 1 - share_data->iterations > 1) {
				//处理GMC，获取全局优化函数
				views_voxels_GMC* max_cover_solver = new views_voxels_GMC(share_data->num_of_max_flow_node, now_view_space, *now_views_infromation, nbv_plan->voxel_information, share_data);
				max_cover_solver->solve();
				vector<pair<int, int>> coverage_view_id_voxelnum_set = max_cover_solver->get_view_id_voxelnum_set();
				num_of_cover = coverage_view_id_voxelnum_set.size();
				for (int i = 0; i < now_view_space->views.size(); i++)
					now_view_space->views[i].in_cover = 0;
				for (int i = 0; i < coverage_view_id_voxelnum_set.size(); i++){
					now_view_space->views[coverage_view_id_voxelnum_set[i].first].in_cover = coverage_view_id_voxelnum_set[i].second;
					num_of_voxel += coverage_view_id_voxelnum_set[i].second;
				}
				delete max_cover_solver;
			}
			else {
				for (int i = 0; i < now_view_space->views.size(); i++)
					now_view_space->views[i].in_cover = 0;
			}
			//保证分母不为0，无实际意义
			num_of_voxel = max(num_of_voxel, 1);
		}
		else if (share_data->method_of_IG == OursIG) {
			//处理网络流，获取全局优化函数
			views_voxels_MF* set_cover_solver = new views_voxels_MF(share_data->num_of_max_flow_node, now_view_space, *now_views_infromation, nbv_plan->voxel_information, share_data);
			set_cover_solver->solve();
			vector<int> coverage_view_id_set = set_cover_solver->get_view_id_set();
			for (int i = 0; i < coverage_view_id_set.size(); i++)
				now_view_space->views[coverage_view_id_set[i]].in_coverage[iterations] = 1;
			//no his
			if (share_data->his_on == false) {
				//初值为1，保证分母不为0，无实际意义
				num_of_cover = max(num_of_cover, (int)coverage_view_id_set.size());
				//num_of_cover = coverage_view_id_set.size();
				for (int i = 0; i < now_view_space->views.size(); i++)
					now_view_space->views[i].in_cover = 0;
				for (int i = 0; i < coverage_view_id_set.size(); i++)
					now_view_space->views[coverage_view_id_set[i]].in_cover = 1;
			}
			delete set_cover_solver;
		}
		else if (share_data->method_of_IG == OursSCOP) {
			//处理SCOP，获取全局优化函数
			views_voxels_LM* set_cover_solver = new views_voxels_LM(share_data->num_of_max_flow_node, now_view_space, *now_views_infromation, nbv_plan->voxel_information, share_data);
			set_cover_solver->solve();
			vector<int> coverage_view_id_set = set_cover_solver->get_view_id_set();
			//初值为1，保证分母不为0，无实际意义
			num_of_cover = max(num_of_cover, (int)coverage_view_id_set.size());
			//num_of_cover = coverage_view_id_set.size();
			for (int i = 0; i < now_view_space->views.size(); i++)
				now_view_space->views[i].in_cover = 0;
			for (int i = 0; i < coverage_view_id_set.size(); i++)
				now_view_space->views[coverage_view_id_set[i]].in_cover = 1;
			delete set_cover_solver;
		}
		//综合计算局部贪心与全局优化，产生视点信息熵
		share_data->sum_local_information = 0;
		share_data->sum_global_information = 0;
		share_data->sum_robot_cost = 0;
		for (int i = 0; i < now_view_space->views.size(); i++) {
			share_data->sum_local_information += now_view_space->views[i].information_gain;
			share_data->sum_global_information += now_view_space->views[i].get_global_information();
			share_data->sum_robot_cost += now_view_space->views[i].robot_cost;
		}
		//保证分母不为0，无实际意义
		if (share_data->sum_local_information == 0) share_data->sum_local_information = 1.0;
		if (share_data->sum_global_information == 0) share_data->sum_global_information = 1.0;
		for (int i = 0; i < now_view_space->views.size(); i++) {
			if (share_data->method_of_IG == OursIG) {
				now_view_space->views[i].final_utility = (1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].get_global_information() / share_data->sum_global_information;
				if (share_data->his_on == false) now_view_space->views[i].final_utility = (1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].in_cover / num_of_cover;
				if (share_data->move_cost_on == true) now_view_space->views[i].final_utility = (1 - share_data->move_weight) * ((1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].get_global_information() / share_data->sum_global_information) + share_data->move_weight * (share_data->robot_cost_negtive == true ? -1 : 1) * now_view_space->views[i].robot_cost / share_data->sum_robot_cost;
			}
			else if (share_data->method_of_IG == GMC) now_view_space->views[i].final_utility = (1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].in_cover / num_of_voxel;
			else if (share_data->method_of_IG == OursSCOP) now_view_space->views[i].final_utility = (1 - share_data->cost_weight) * now_view_space->views[i].information_gain / share_data->sum_local_information + share_data->cost_weight * now_view_space->views[i].in_cover / num_of_cover;
			else if (share_data->method_of_IG == APORA) now_view_space->views[i].final_utility = now_view_space->views[i].information_gain;
			else if (share_data->method_of_IG == RSE) now_view_space->views[i].final_utility = now_view_space->views[i].information_gain;
			else if (share_data->method_of_IG == OursBasic) now_view_space->views[i].final_utility = now_view_space->views[i].information_gain;
			else now_view_space->views[i].final_utility = 0.5 * (share_data->sum_local_information == 0 ? 0 : now_view_space->views[i].information_gain / share_data->sum_local_information) + 0.5 * (share_data->robot_cost_negtive == true ? -1 : 1) * now_view_space->views[i].robot_cost / share_data->sum_robot_cost;
		}
	}
	//更新标志位
	share_data->now_views_infromation_processed = true;
}

void move_robot(View* now_best_view, View_Space* now_view_space, Share_Data* share_data) {
	if (share_data->num_of_max_iteration > 0 && nbv_plan->iterations + 1 >= share_data->num_of_max_iteration) share_data->over = true;
	if (!share_data->move_wait) share_data->move_on = true;
}

void show_cloud(pcl::visualization::PCLVisualizer::Ptr viewer) {
	//pcl显示点云
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void get_command()
{	//从控制台读取指令字符串
	string cmd;
	while (!stop && !share_data->over)
	{
		cout << "Input command 1.stop 2.over 3.next_itreation :" << endl;
		cin >> cmd;
		if (cmd == "1") stop = true;
		else if (cmd == "2") share_data->over = true;
		else if (cmd == "3") share_data->move_on = true;
		else cout << "Wrong command.Retry :" << endl;
	}
	cout << "get_command over." << endl;
}

void get_run()
{
	//NBV规划期初始化
	nbv_plan = new NBV_Planner(share_data);
	//主控循环
	string status="";
	//实时读取与规划
	while (!stop && nbv_plan->plan()) {
		//如果状态有变化就输出
		if (status != nbv_plan->out_status()) {
			status = nbv_plan->out_status();
			cout << "NBV_Planner's status is " << status << endl;
		}
	}
	delete nbv_plan;
}

#define DebugOneObject 0
#define TestOneMethod 1
#define DebugOurMethod 2
#define GetGTPoints 3
#define TryGTSize 4
#define HandGTSize 5

int mode = GetGTPoints;

int main()
{
	//Init
	ios::sync_with_stdio(false);
	cout << "input mode:";
	cin >> mode;
	vector<int> first_view_ids;
	first_view_ids.push_back(0);
	first_view_ids.push_back(1);
	first_view_ids.push_back(2);
	first_view_ids.push_back(3);
	first_view_ids.push_back(4);
	//选取模式
	if (mode == DebugOneObject)
	{
		//数据区初始化
		share_data = new Share_Data("../DefaultConfiguration.yaml");
		//控制台读取指令线程
		//thread cmd(get_command);
		//NBV系统运行线程
		thread runner(get_run);
		//等待线程结束
		runner.join();
		//cmd.join();
		delete share_data;
	}
	else if (mode == TestOneMethod){
		//测试集
		vector<string> names;
		cout << "input models:" << endl;
		string name;
		while (cin >> name) {
			if (name == "-1") break;
			names.push_back(name);
		}
		int method_id;
		cout << "thread for method id:";
		cin >> method_id;
		//测试
		for (int i = 0; i < names.size(); i++) {
			for (int k = 0; k < first_view_ids.size(); k++) {
				//数据区初始化
				share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], first_view_ids[k], -1, -1, method_id);
				//NBV系统运行线程
				thread runner(get_run);
				//等待线程结束
				runner.join();
				delete share_data;
			}
		}
	}
	else if (mode == DebugOurMethod) {
		//测试集
		vector<string> names;
		cout << "input models:" << endl;
		string name;
		while (cin >> name) {
			if (name == "-1") break;
			names.push_back(name);
		}
		for (int i = 0; i < names.size(); i++) {
			for (int k = 0; k < first_view_ids.size(); k++) {
				for (double visble_rate = 0.1; visble_rate <= 1.01; visble_rate += 0.1) //if (fabs(visble_rate - 0.3) < 1e-6)
				{
					for (double move_rate = 1.0; move_rate <= 1.01; move_rate += 0.1)
					{
						for (double skip_threshold = 0.2; skip_threshold <= 0.21; skip_threshold += 0.2)
						{
							cout << "now test is " << names[i] << " , " << visble_rate << " , " << move_rate << " , " << skip_threshold << endl;
							share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], first_view_ids[k], visble_rate, move_rate, 10, -1, skip_threshold);
							thread runner(get_run);
							runner.join();
							delete share_data;
						}
					}
				}
			}
		}
	}
	else if (mode == GetGTPoints) {
		//测试集
		vector<string> names;
		cout << "input models:" << endl;
		string name;
		while (cin >> name) {
			if (name == "-1") break;
			names.push_back(name);
		}
		for (int i = 0; i < names.size(); i++) {
			//数据区初始化
			share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], -1, -1, -1, 6);
			//NBV规划期初始化
			nbv_plan = new NBV_Planner(share_data);
			//获取全部点云
			for (int i = 0; i < nbv_plan->now_view_space->views.size(); i++) {
				nbv_plan->percept->precept(&nbv_plan->now_view_space->views[i]);
			}
			//根据Octomap精度统计可见点个数
			double now_time = clock();
			int num = 0;
			unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>();
			for (int j = 0; j < share_data->cloud_final->points.size(); j++) {
				octomap::OcTreeKey key = share_data->ground_truth_model->coordToKey(share_data->cloud_final->points[j].x, share_data->cloud_final->points[j].y, share_data->cloud_final->points[j].z);
				if (voxel->find(key) == voxel->end()) {
					(*voxel)[key] = num++;
				}
			}
			cout << "GT visible pointcloud get with executed time " << clock() - now_time << " ms." << endl;
			//保存GT点云分辨率体素网格，红色为可见
			share_data->access_directory(share_data->pre_path + "/Ground_truth/");
			//保存GT
			share_data->access_directory(share_data->pre_path + share_data->vs_path + "/GT_points_num/");
			ofstream fout_GT_points_num(share_data->pre_path + share_data->vs_path + "/GT_points_num/" + names[i] + ".txt");
			fout_GT_points_num << voxel->size() <<'\t'<< share_data->cloud_points_number << endl;
			cout << "GT_points_num is " << voxel->size() <<" and cloud num is "<< share_data->cloud_points_number << endl;
			delete voxel;
			delete nbv_plan;
			delete share_data;
		}
	}
	else if (mode == TryGTSize) {
		//测试集
		vector<string> names;
		cout << "input models:" << endl;
		string name;
		while (cin >> name) {
			if (name == "-1") break;
			names.push_back(name);
		}
		for (int i = 0; i < names.size(); i++) {
			cout << "Get GT size of pointcloud model " << names[i] << endl;
			bool coveraged = false;
			double init_size = -1;
			//if (names[i] == "Thai_Statue" || names[i] == "Lucy") init_size = 0.20109;
			ifstream fin_GT_size("E:/GMC/Ground_size/" + names[i] + ".txt");
			if (fin_GT_size.is_open()) {
				fin_GT_size >> init_size;
			}
			share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], -1, -1, -1, 6, init_size);
			nbv_plan = new NBV_Planner(share_data);
			vector<double> visible_nums;
			int num = 0;
			unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>* voxel = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>();
			double now_time = clock();
			int pre_visible_nums = -1;
			for (int i = 0; i < nbv_plan->now_view_space->views.size(); i++) {
				nbv_plan->percept->precept(&nbv_plan->now_view_space->views[i]);
				//每次都根据统计一下Octomap精度统计可见点个数
				for (int j = 0; j < share_data->clouds[i]->points.size(); j++) {
					octomap::OcTreeKey key = share_data->ground_truth_model->coordToKey(share_data->clouds[i]->points[j].x, share_data->clouds[i]->points[j].y, share_data->clouds[i]->points[j].z);
					if (voxel->find(key) == voxel->end()) {
						(*voxel)[key] = num++;
					}
				}
				visible_nums.push_back(num);
				if ((i + 9) % 50 == 0) {
					cout << "Viewsapce size " << i << " has visible points " << num << endl;
					if (num == pre_visible_nums) {
						coveraged = true;
						//break;
					}
					pre_visible_nums = num;
				}
			}
			cout << "GT visible pointcloud get with executed time " << clock() - now_time << " ms." << endl;
			/*double left = 0.0;
			double right = nbv_plan->now_view_space->predicted_size;
			cout << "original size is " << nbv_plan->now_view_space->predicted_size << endl;
			double mid = nbv_plan->now_view_space->predicted_size;
			while (!coveraged) {
				remove((share_data->pre_path + share_data->vs_path + "/viewspace/" + share_data->name_of_pcd + ".txt").c_str());
				delete voxel;
				delete nbv_plan;
				delete share_data;
				mid = (left + right) / 2.0;
				cout << "test size is " << mid << endl;
				share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], -1, -1, -1, 6, mid);
				nbv_plan = new NBV_Planner(share_data);
				visible_nums.clear();
				voxel = new unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash>();
				int num = 0;
				double now_time = clock();
				int pre_visible_nums = -1;
				for (int i = 0; i < nbv_plan->now_view_space->views.size(); i++) {
					nbv_plan->percept->precept(&nbv_plan->now_view_space->views[i]);
					//每次都根据统计一下Octomap精度统计可见点个数
					for (int j = 0; j < share_data->clouds[i]->points.size(); j++) {
						octomap::OcTreeKey key = share_data->ground_truth_model->coordToKey(share_data->clouds[i]->points[j].x, share_data->clouds[i]->points[j].y, share_data->clouds[i]->points[j].z);
						if (voxel->find(key) == voxel->end()) {
							(*voxel)[key] = num++;
						}
					}
					visible_nums.push_back(num);
					if ((i + 9) % 50 == 0) {
						cout << "Viewsapce size " << i << " has visible points " << num << endl;
						if (num == pre_visible_nums) {
							coveraged = true;
							break;
						}
						pre_visible_nums = num;
					}
				}
				cout << "GT visible pointcloud get with executed time " << clock() - now_time << " ms." << endl;
				if (coveraged) {
					left = mid;
					if (left + 0.005 >= right) break;
					coveraged = false;
				}
				else {
					right = mid;
				}
			}
			*/
			//保存GT点云分辨率体素网格，红色为可见
			share_data->access_directory(share_data->pre_path + share_data->vs_path + "/Ground_truth/");
			for (auto it = voxel->begin(); it != voxel->end(); it++) {
				share_data->ground_truth_model->setNodeColor(it->first, 255, 0, 0);
			}
			share_data->ground_truth_model->write(share_data->pre_path + share_data->vs_path + "/Ground_truth/" + names[i] + ".ot");
			//保存GT
			share_data->access_directory(share_data->pre_path + share_data->vs_path + "/Ground_truth/");
			ofstream fout_GT_points_num(share_data->pre_path + share_data->vs_path + "/Ground_truth/" + names[i] + ".txt");
			for (int i = 0; i < visible_nums.size(); i++) {
				fout_GT_points_num << i << '\t' << visible_nums[i] << '\n';
			}
			fout_GT_points_num << visible_nums[visible_nums.size() - 1] << "\t" << 1.0 * visible_nums[visible_nums.size() - 1] / share_data->cloud_points_number << endl;
			//fout_GT_points_num << visible_nums[visible_nums.size() - 1] << "\t" << 1.0 * visible_nums[visible_nums.size() - 1] / share_data->cloud_points_number << '\t' << mid << endl;
			cout << "GT_points_num is " << visible_nums[visible_nums.size() - 1] << ". Rate is " << 1.0 * visible_nums[visible_nums.size() - 1] / share_data->cloud_points_number << endl;
			//cout << "GT_points_num is " << visible_nums[visible_nums.size() - 1] << ". Rate is " << 1.0 * visible_nums[visible_nums.size() - 1] / share_data->cloud_points_number << " Size is " << mid << endl;
			delete voxel;
			delete nbv_plan;
			delete share_data;
		}
	}
	else if (mode == HandGTSize) {
		//测试集
		vector<string> names;
		cout << "input models:" << endl;
		string name;
		while (cin >> name) {
			if (name == "-1") break;
			names.push_back(name);
		}
		for (int i = 0; i < names.size(); i++) {
			cout << "Get GT size of pointcloud model " << names[i] << endl;
			double init_size = -1;
			if (names[i] == "Thai_Statue" || names[i] == "Lucy") init_size = 0.20109;
			share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], -1, -1, -1, 6, init_size);
			nbv_plan = new NBV_Planner(share_data);
			double original_size = nbv_plan->now_view_space->predicted_size;
			init_size = nbv_plan->now_view_space->predicted_size;
			share_data->access_directory(share_data->pre_path + "/Ground_size/");
			for (auto it = share_data->ground_truth_model->begin_leafs(); it != share_data->ground_truth_model->end_leafs(); it++) {
				share_data->ground_truth_model->setNodeColor(it.getKey(), 255, 0, 0);
			}
			share_data->ground_truth_model->write(share_data->pre_path + "/Ground_size/" + names[i] + ".ot");
			bool ok;
			cout << "Minus 0.01m (1/0):";
			cin >> ok;
			while (ok) {
				init_size -= 0.01;
				delete nbv_plan;
				delete share_data;
				share_data = new Share_Data("../DefaultConfiguration.yaml", names[i], -1, -1, -1, 6, init_size);
				nbv_plan = new NBV_Planner(share_data);
				share_data->access_directory(share_data->pre_path + "/Ground_size/");
				for (auto it = share_data->ground_truth_model->begin_leafs(); it != share_data->ground_truth_model->end_leafs(); it++) {
					share_data->ground_truth_model->setNodeColor(it.getKey(), 255, 0, 0);
				}
				share_data->ground_truth_model->write(share_data->pre_path + "/Ground_size/" + names[i] + ".ot");
				cout << "Minus 0.01m (1/0):";
				cin >> ok;
			};
			if (original_size != init_size) {
				share_data->access_directory(share_data->pre_path + "/Ground_size/");
				ofstream fout_GT_size(share_data->pre_path + "/Ground_size/" + names[i] + ".txt");
				fout_GT_size << init_size << '\n';
			}
		}
	}
	cout << "System over." << endl;
	return 0;
}

/*
Armadillo
Dragon
Stanford_Bunny
Happy_Buddha
Thai_Statue
Lucy
LM1
LM2
LM3
LM4
LM5
LM6
LM7
LM8
LM9
LM10
LM11
LM12
obj_000001
obj_000002
obj_000003
obj_000004
obj_000005
obj_000006
obj_000007
obj_000008
obj_000009
obj_000010
obj_000011
obj_000012
obj_000013
obj_000014
obj_000015
obj_000016
obj_000017
obj_000018
obj_000019
obj_000020
obj_000021
obj_000022
obj_000023
obj_000024
obj_000025
obj_000026
obj_000027
obj_000028
obj_000029
obj_000030
obj_000031
*/