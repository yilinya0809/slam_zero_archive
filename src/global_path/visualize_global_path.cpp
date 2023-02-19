#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include "ros/ros.h"
#include "ros/package.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

enum VisTarget{
	FLAG,
	SPEED,
	CURVE
};
// what to visualize
bool is_kcity = false;
bool is_delivery = true;
// draw individual path or all in one
bool is_all_in_one = false;
int target_flag = 13;
// draw speed vs draw flag
VisTarget target = FLAG;

void XYToPixel(double& pixel_x, double& pixel_y, double x, double y, bool is_kcity) {
    if (is_kcity) {
        double a1 = 33.30845062530643;
        double b1 = 0.006115022759639878;
        double c1 = -10098174.098572133;
        double a2 = 0.006115022759636954;
        double b2 = -33.30845062530642;
        double c2 = 137371264.71873185;

        pixel_x = a1*x+b1*y+c1;
        pixel_y = a2*x+b2*y+c2;
    } else {
        double a1 = 33.35682000061858;
        double b1 = -0.6892112032054343;
        double c1 = -7095938.941479745;
        double a2 = -0.6892112032054311;
        double b2 = -33.35682000061859;
        double c2 = 138233386.34684;

        pixel_x = a1*x+b1*y+c1;
        pixel_y = a2*x+b2*y+c2;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "visualizeglobalpath");

	std::stringstream img_path;
	if (!is_kcity) {
		img_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_costmap.png";
	}
	else {
		img_path << ros::package::getPath("slam") << "/config/KCity/KCity_costmap.png";
	}

	std::stringstream path_path;
	if (!is_kcity && !is_delivery) {
		path_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
	}
	else if (!is_kcity && is_delivery) {
		path_path << ros::package::getPath("slam") << "/config/FMTC/delivery.txt";
	}
	else if (is_kcity) {
		path_path << ros::package::getPath("slam") << "/config/KCity/KCity_global_path.txt";
	}
	else  {
		ROS_INFO("INVALID PATHSTREAM!!!!!!!!!!!!!!!");
	}
	
	// load image
	cv::Mat glob_img = cv::imread(img_path.str(), cv::IMREAD_COLOR);
	if (glob_img.empty()) {
		ROS_INFO("img load error");
	}
	else {
		ROS_INFO("img loaded");
	}

	// read path
	std::ifstream file;
	file.open(path_path.str().c_str());
	
	std::string line;
	std::string token;

	std::vector<double> utm_x;
	std::vector<double> utm_y;
	std::vector<int> flag;
	std::vector<double> speed;
	std::vector<int> curve;
	size_t length;

	int linecnt = 0;
	while (std::getline(file, line)) {
		//std::cout<<linecnt++<<std::endl;
		std::stringstream ss(line);
		int cnt = 0;
		while (std::getline(ss, token, ' ')) {
			switch (cnt++)
			{
				case 0:
					utm_x.push_back(std::stod(token));
				break;
				case 1:  
					utm_y.push_back(std::stod(token));
				break;
				case 2:
					flag.push_back(std::stoi(token));
				break;
				case 3:
					speed.push_back(std::stod(token));
				break;
				case 4:
					curve.push_back(std::stod(token));
				break;
				default:
				break;
			}
		}
	}
	length = utm_x.size();
	file.close();

	if (utm_x.empty()) {
		ROS_INFO("path load error");
	}
	else {
		ROS_INFO("path loaded");
	}

	std::vector<double> x;
	std::vector<double> y;
	for (int i=0; i<length; i++) {
		double tmp_x, tmp_y;
		XYToPixel(tmp_x, tmp_y, utm_x[i], utm_y[i], is_kcity);
		x.push_back(tmp_x);
		y.push_back(tmp_y);
	}
	int count = 0;
	if (is_all_in_one) {
		for (int i=0; i<length; i++) {
			// print point position on the image
			cv::Point position(x[i], y[i]);
			if (count == 10) {
				std::string text = std::to_string(i);
				cv::putText(glob_img, text, position, 1, 0.8, cv::Scalar::all(255));
				count = 0;
			}
			else {
				count++;
			}

			if (target == SPEED) {
				cv::circle(glob_img, position, 1, cv::Scalar(0, 0, 50*speed[i]), -1);
			}
			else if (target == FLAG) {
				cv::circle(glob_img, position, 1, cv::Scalar((flag[i]*500) % 255, (flag[i]*100) % 255, 255 - (flag[i]*100) % 255), -1);
			}
			else if (target == CURVE) {
				cv::circle(glob_img, position, 1, cv::Scalar(250*curve[i], 0, 255), -1);	
			}
			else {}		
		}
	}
	else {
		for (int i=0; i<length; i++) {
			cv::Point position(x[i], y[i]);
			if (count == 10) {
				if (flag[i] == target_flag) {
					std::string text = std::to_string(i);
					cv::putText(glob_img, text, position, 1, 0.8, cv::Scalar::all(255));
					if (target == SPEED) {
						cv::circle(glob_img, cv::Point(x[i], y[i]), 1, cv::Scalar(0, 0, 50*speed[i]), -1);
					}
					else if (target == FLAG) {
						cv::circle(glob_img, cv::Point(x[i], y[i]), 1, cv::Scalar((flag[i]*500) % 255, (flag[i]*100) % 255, 255 - (flag[i]*100) % 255), -1);
					}
					else if (target == CURVE) {
						cv::circle(glob_img, cv::Point(x[i], y[i]), 1, cv::Scalar(250*curve[i], 0, 255), -1);	
					}
					else {}
				}
				else {}
				count = 0;
			}
			else {
				count++;
			}
		}
	}
	cv::imwrite("/home/cjsrmswp/catkin_ws/visualized.png", glob_img);	
	return 0;
}
 
