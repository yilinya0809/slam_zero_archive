#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Path.h"

#include "slam/Cluster.h"
#include "slam/Clusters.h"
#include "slam/Lidar.h"
#include "slam/LidarPoint.h"
#include "slam/Data.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

class ConeDetector{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_count_;
    ros::Publisher pub_center_;
    ros::Publisher pub_clus_;

    ros::Subscriber sub_lidar_;

    vector<slam::LidarPoint> cloud_points_;
    vector<slam::LidarPoint> filtered_points_;
    vector<int> clustering_helper_;

    double lidar_angle_;
    double lidar_height_;
    double cone_height_;

    double plane_config_[4]; 
    double plane_tolerance_;

    double cluster_tolerance_;

    double out_circle_;
    double in_circle_;
    double width;
    double height;

    int cluster_num_min_;
    int cluster_num_max_;

    double pitch_offset;

    stringstream path_stream_;
    cv::Mat road_map_;

    double max_time = 0.0;

    public:
    ConeDetector(){
        pub_ = nh_.advertise<slam::Clusters>("/cone_point_cloud", 2);
	pub_count_ = nh_.advertise<std_msgs::Int32MultiArray>("cone_count",2);
        pub_center_ = nh_.advertise<sensor_msgs::PointCloud>("cone_center",2);
        pub_clus_ = nh_.advertise<sensor_msgs::PointCloud>("cone_clus", 2);
        
	sub_lidar_ = nh_.subscribe("/points", 1, &ConeDetector::callback_lidar, this);

        lidar_angle_ = 0;
        lidar_height_ = 0.88;
        cone_height_ = 0.75;
        plane_tolerance_ = 0.25;
        
	out_circle_ = 9;
        in_circle_ = 1;
        width = 3; 
        height = 9;

        cluster_tolerance_ = 0.25;
        cluster_num_min_ = 5;
        cluster_num_max_ = 120;
    }


    double get_distance(slam::LidarPoint p1, slam::LidarPoint p2){
        return sqrt((p1.point_2d.x-p2.point_2d.x)*(p1.point_2d.x-p2.point_2d.x)+(p1.point_2d.y-p2.point_2d.y)*(p1.point_2d.y-p2.point_2d.y));
    }

    inline int find(int a){
        if(clustering_helper_[a]<0) return a;
        clustering_helper_[a] = find(clustering_helper_[a]);
        return clustering_helper_[a];
    }

    inline void merge(int a, int b){
        a = find(a);
        b = find(b);
        if(a==b) return;
        clustering_helper_[b] = a;
    }

    void removing_plane(){
        set<int> inliers_result;
        int n = cloud_points_.size();
        for(int i = 0; i<n;i++){
            geometry_msgs::Point p = cloud_points_.at(i).point_3d;
            double point_height = p.z+lidar_height_;
            if (point_height > plane_tolerance_){
                filtered_points_.push_back(cloud_points_.at(i));
            }
        }
    }

    void removing_outside(){
        vector<slam::LidarPoint> inside;
        for(slam::LidarPoint p : filtered_points_){
            double r = sqrt(p.point_3d.x*p.point_3d.x+p.point_3d.y*p.point_3d.y);
            if(fabs(p.point_2d.y) < width && p.point_2d.x > 0.05 && p.point_2d.x < height){
                inside.push_back(p);
                clustering_helper_.push_back(-1);
            }
        }
        filtered_points_ = inside;
    }

    void removing_circle_inout(){
        vector<slam::LidarPoint> inside;
        for(slam::LidarPoint p : filtered_points_){
            double r = sqrt(p.point_3d.x*p.point_3d.x+p.point_3d.y*p.point_3d.y);
            if(r > in_circle_ && r < out_circle_){
                inside.push_back(p);
                clustering_helper_.push_back(-1);
            }
            
        }
        filtered_points_ = inside;
    }

    vector<vector<int>> clustering(){
        int n = filtered_points_.size();
        
        for(int i = 0;i<n;i++){
            for(int j=i+1;j<n;j++){
                if(find(i) == find(j)) continue;
                if(get_distance(filtered_points_.at(i), filtered_points_.at(j)) < cluster_tolerance_){
                    merge(i,j);
                }
            }
        }
        vector<vector<int>> storage;
        vector<vector<int>> clusters;
        for(int i=0;i<n;i++) {
            vector<int> cluster;
            storage.push_back(cluster);
        }
        for(int i=0;i<n;i++){
            storage.at(find(i)).push_back(i);
        }
        for(int i=0;i<n;i++){
            int merge_num = storage.at(i).size();
            if(merge_num < cluster_num_max_ && merge_num > cluster_num_min_) clusters.push_back(storage.at(i));
        }
        return clusters;
    }

    void callback_lidar(const slam::Lidar::ConstPtr& msg){
        cloud_points_ = msg->points;
        filtered_points_.clear();
        clustering_helper_.clear();

        ROS_INFO_STREAM("# of initial points : " << cloud_points_.size());

        removing_plane();
        removing_circle_inout();
        vector<vector<int>> clusters = clustering();
        
        //Publish Data
        slam::Clusters rt;
        std_msgs::Int32MultiArray rt_count;
        sensor_msgs::PointCloud rt_center;

        vector<int> cluster_counts;
        vector<slam::Cluster> rt_clusters;
        vector<geometry_msgs::Point32> rt_centers;
        rt.header.stamp = ros::Time::now();
        
        rt_center.header.frame_id = "velodyne";
        int higher_point;
        geometry_msgs::Point32 center;
        
	slam::LidarPoint pi;
        bool outside_flag;
        double x, y, RLflag = 0;

	sensor_msgs::PointCloud rt_clus;
        rt_clus.header.frame_id = "velodyne";
        vector<geometry_msgs::Point32> rt_cluses;
	geometry_msgs::Point32 clus;

        for(vector<int> cluster : clusters){
            slam::Cluster rt_cluster;
            x = 0; y = 0;
            rt_cluster.count = 0;
            higher_point = 0;
            outside_flag = false;
            for(int index : cluster){
                pi = filtered_points_.at(index);
                
                if(pi.channel >= 7){ //down 14 up 7
                    if(higher_point > 5) break;
                    higher_point ++;
                    continue;
                }
                if(fabs(pi.point_2d.y) > width || pi.point_2d.x < 0.05 || pi.point_2d.x > height){
                    outside_flag = true;
                    break;
                }

	        clus.x=pi.point_3d.x;
	        clus.y=pi.point_3d.y;
                clus.z=pi.point_3d.z;                         
		rt_cluses.push_back(clus);
                
                x += pi.point_3d.x;
                y += pi.point_3d.y;
                
                rt_cluster.points.push_back(pi);
                rt_cluster.count ++;
            }
            
            if(rt_cluster.count == 0) 
            {
                ROS_INFO_STREAM("zero count points info : " << center);
                continue;
            }
            x /= rt_cluster.count;
            y /= rt_cluster.count;

            if(y >= 0) RLflag = 0; // YELLOW == 0(LEFT)
            else if (y < 0) RLflag = 1; // BLUE == 1(RIGHT)

            center.x = x;
            center.y = y;
            center.z = RLflag;

            if(outside_flag || higher_point > 5) continue;

            rt_centers.push_back(center);
            rt_clusters.push_back(rt_cluster);
            cluster_counts.push_back(rt_cluster.count);
        }

        rt.clusters = rt_clusters;
        rt_count.data = cluster_counts;
        rt_center.points = rt_centers;
	rt_clus.points = rt_cluses;

        pub_.publish(rt);
        pub_count_.publish(rt_count);
        pub_center_.publish(rt_center);
	pub_clus_.publish(rt_clus);

        ROS_INFO_STREAM("# of filtered points : " << filtered_points_.size());
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "cone_detector");
    ConeDetector cone_detector;
    ros::spin();
}
