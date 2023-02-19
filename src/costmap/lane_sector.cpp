#include <iostream>
#include <string>
#include <vector>

#include "std_msgs/UInt32.h"

#include "slam/Data.h"
#include "slam/Pixel.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"


class lane_sector_publisher{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        int nBlue, nGreen, nRed;
        bool is_kcity;

    public:
        std::stringstream path_stream;
        cv::Mat color_map;

        //cv::Mat color_map = cv::imread("/home/healthykim/catkin_ws/src/zero/slam/config/KCity/KCity_color_map.png", cv::IMREAD_COLOR);

        lane_sector_publisher() {
            ros::param::get("/is_kcity", is_kcity);
	        is_kcity=false;
            if(is_kcity==true){
                path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_map_for_lanesector.png";
                color_map = cv::imread(path_stream.str());
                if(!color_map.empty()){
                    ROS_INFO("KCity loaded");
                }  
            }
            else if(is_kcity==false){
                path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map_for_lanesector.png";
                color_map = cv::imread(path_stream.str()); 
                if(!color_map.empty()){
                    ROS_INFO("FMTC loaded");
                }     
            }  
            pub = nh.advertise<std_msgs::UInt32>("/lane_state", 2);
            sub = nh.subscribe("/filtered_data",2, &lane_sector_publisher::callback, this);
        }

        //void callback(const slam::Pixel Data){
        void callback(const slam::Data::ConstPtr& msg){
            bool x_inRange, y_inRange;
	    static int lane_state{0};

            XYToPixel(pixel_y, pixel_x, msg->x, msg->y, is_kcity); // pixel_y here is x in cv graphics and column in cv Mat
            
            if(is_kcity==true){
            x_inRange ={pixel_x<=22489 && pixel_x > 0};
            y_inRange ={pixel_y<=8273 && pixel_y > 0};
            }

            if(is_kcity==false){
            x_inRange ={pixel_x<=14226 && pixel_x > 0};
            y_inRange ={pixel_y<=12072 && pixel_y > 0};
            }

            std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y <<std::endl;

            if(x_inRange&&y_inRange){
                std::cout<<"on map"<<std::endl;
                //get the RGB information of a pixel 
                nBlue = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
                nGreen = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[1];
                nRed = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[2];
                
                std::cout<<nBlue<<" "<<nGreen<<" "<<nRed<<std::endl;

                if(nBlue==0 && nGreen==0 && nRed==0)
                    lane_state = 0;

		        else
		        lane_state = 1;
             
                std_msgs::UInt32 rt;
		        rt.data=lane_state;
                pub.publish(rt);
                 //previous_data = rt.data;
                 
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lane_sector_publisher");
    lane_sector_publisher lane_sector_publisher;
    ros::spin();
}
