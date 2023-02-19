#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt32.h"

#include "slam/Data.h"

#include "ros/package.h"
#include "ros/time.h"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"

using namespace std;
using namespace cv;

/*
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/

//std::stringstream ss;
//ss << ros::package::getPath("slam") << "src/mapping/costmap.png";

class Lane_costmap_publisher{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber costmap_sub;
		ros::Subscriber gear_state_sub;
		ros::Subscriber lane_state_sub;
			
		bool is_kcity;

	public:
		int map_size = 300;
		std::stringstream path_stream;
		cv::Mat glob_costmap;
		int gear_state{0};
		int lane_state{0};
		int mission_state{0};
			
		//Constructor for lane_costmap
		Lane_costmap_publisher() {
			ros::param::get("/is_kcity", is_kcity);
			is_kcity=false;

			if(!is_kcity) path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map_for_costmap.png";
			//if(!is_kcity) path_stream << ros::package::getPath("slam") << "/config/FMTC/empty_costmap.png";
			else path_stream << ros::package::getPath("slam") << "/config/KCity/KCity.png";
			glob_costmap = cv::imread(path_stream.str(), cv::IMREAD_GRAYSCALE);
						
			ROS_INFO("Image loaded");

			pub = nh.advertise<sensor_msgs::Image>("/lane_costmap", 2);
			costmap_sub = nh.subscribe("/filtered_data", 1, &Lane_costmap_publisher::callback, this);
			gear_state_sub = nh.subscribe("/gear_state", 1, &Lane_costmap_publisher::gs_callback, this);
			lane_state_sub = nh.subscribe("/lane_state", 1, &Lane_costmap_publisher::ls_callback, this);			
		}
			
		int max(int a, int b){ return (a>b)?a:b;}

		void gs_callback(const std_msgs::UInt32 state){
			gear_state = state.data;
			std::cout << gear_state << std::endl;
		}

		void ls_callback(const std_msgs::UInt32 state){
			lane_state = state.data;
			std::cout << "lane state : " << lane_state << std::endl;
		}

		void callback(const slam::Data data){
			clock_t begin = clock();
			cv::Mat lane_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			geometry_msgs::PoseStamped loc_pose;
			int curr_pixel_x{}, curr_pixel_y{};
			double step = 0.5;
			double pix_heading{};
						
			if(data.theta >= 0) pix_heading = data.theta;
			else pix_heading = data.theta + 2*M_PI;

			double head_coor_x, head_coor_y;
			head_coor_x = (step)*sin(pix_heading);
			head_coor_y = (step)*cos(pix_heading);
			XYToPixel(curr_pixel_x, curr_pixel_y, data.x, data.y, is_kcity);
						
			double point_pixel_x{}, point_pixel_y{};

			//forward driving
			//cout << 1 << endl;
			if(!gear_state && lane_state == 0){
				for(int j=1; j<600; j++){
					cout << 2 << endl;
					point_pixel_x = curr_pixel_x + j*head_coor_y;
					point_pixel_y = curr_pixel_y - j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x += head_coor_x;
						point_pixel_y += head_coor_y;
						lane_costmap.at<uchar>(int(300-j*step),int(150+i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}
				cout << 3 << endl;
				for(int j=1; j<600; j++){
					point_pixel_x = curr_pixel_x + j*head_coor_y;
					point_pixel_y = curr_pixel_y - j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x += -head_coor_x;
						point_pixel_y += -head_coor_y;

						lane_costmap.at<uchar>(int(300-j*step),int(150-i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}
				
			}
			/*
			else{
				Mat white_line;
				lane_costmap.copyTo(white_line);
				line(white_line, Point(150, 100), Point(150, 200), Scalar::all(255), 1, 8, 0);
				//line( Input Output Array, pt1, pt2, scalar(b,g,r), thickness, lineType,  shift ) 
			}*/			
		
			ROS_INFO("Publish lane cost map");

			nav_msgs::OccupancyGrid cost_map;
			cost_map.info.width = 300;
			cost_map.info.height = 300;

			for (int i = 1; i < 301; i++){
				for (int j = 1; j < 301; j++) cost_map.data.push_back(static_cast<int8_t>(lane_costmap.at<uchar>(300-i,300-j)));
			}
						
			//cv::imwrite("/home/djlee/asdlfkjal.png", lane_costmap);	
			//change directory		

			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, lane_costmap);
			img_bridge.toImageMsg(img_msg);
			clock_t end = clock();
			pub.publish(img_msg);
			ROS_INFO("elaspsed time : %lf", double(end-begin)/CLOCKS_PER_SEC);
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "lane_costmap_publisher");
        ros::Time::init();
        ros::Rate rate(20); //ROS rate at ??Hz
        Lane_costmap_publisher lane_costmap_publisher;
        while(ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
}


