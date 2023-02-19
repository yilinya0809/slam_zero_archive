#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <sensor_msgs/point_cloud_conversion.h>
#include <velodyne_pointcloud/point_types.h>
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "slam/Lidar.h"
#include "slam/LidarPoint.h"

#include "ros/ros.h"
#include "boost/foreach.hpp"
#include <deque>

using namespace std;

class Lidar_Decoder{
    public:
    Lidar_Decoder(){
        pub_ = n_.advertise<slam::Lidar>("/points", 1);
        sub_ = n_.subscribe("/velodyne_points", 2, &Lidar_Decoder::callback, this);
        latest_mode_time = ros::Time::now();
        distance_tolerance = 12.0;
        lidar_angle = 0;
        lidar_height = 0.90;
        gps_pose = 0.25;
    }	

    void callback(const sensor_msgs::PointCloud2& msg){
        ros::Time t0 = ros::Time::now();
        slam::Lidar rt;
        int count = 0;

        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> vlp_xyzir;
        pcl::PointCloud<pcl::PointXYZI> pcl_xyzi;
        pcl::fromROSMsg(msg, vlp_xyzir);
        pcl::fromROSMsg(msg, pcl_xyzi);

        
        int cloudSize = pcl_xyzi.points.size();
        //cout << cloudSize << endl;
        vector<slam::LidarPoint> cloud_points;

        for(int i = 0; i < cloudSize; i++){
            pcl::PointXYZI point = pcl_xyzi.points[i];
            slam::LidarPoint p;
	    p.point_2d.x = point.x + gps_pose;
            //p.point_2d.x = point.x*cos(lidar_angle*M_PI/180) + point.z*sin(lidar_angle*M_PI/180) + gps_pose;
            p.point_2d.y = point.y;
            p.point_2d.z = 0;
            p.point_3d.x = point.x;
            p.point_3d.y = point.y;
            p.point_3d.z = point.z;
            p.channel = vlp_xyzir.points[i].ring;
            
            if(p.point_2d.x > 0.5 && point.intensity > 10 && sqrt(p.point_2d.x*p.point_2d.x+p.point_2d.y*p.point_2d.y) < distance_tolerance){
                cloud_points.push_back(p);
                count++;
            }
        }
        
        rt.header =  msg.header;
        rt.header.stamp = ros::Time::now();
        rt.count = count;
        rt.points = cloud_points;
	pub_.publish(rt);
    }

    private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Time latest_mode_time;
    double distance_tolerance;
    double lidar_angle;
    double lidar_height;
    double gps_pose;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_decoder");
    Lidar_Decoder LidarDecoder;
    ros::spin();
}
