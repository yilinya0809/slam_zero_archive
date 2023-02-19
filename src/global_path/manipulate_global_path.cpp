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

// what to manipulate
bool is_kcity = true;
bool is_pre = false;

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

void PixelToXY(double& x, double& y, double pixel_x, double pixel_y, bool is_kcity) {
    if (is_kcity) {
        double a1 = 0.030022410154851;
        double b1 = 5.51174605692027E-06;
        double c1 = 10098174.098572133;
        double a2 = 5.51174605691764E-06;
        double b2 = -0.030022410154851;
        double c2 = -137371264.71873185;

        pixel_x += c1; pixel_y += c2;
        x = a1*pixel_x+b1*pixel_y;
        y = a2*pixel_x+b2*pixel_y;
    } else {
        double a1 = 0.02996608409;
        double b1 = -0.000619152571;
        double c1 = 7095938.941479745;
        double a2 = -0.000619152571;
        double b2 = -0.02996608409;
        double c2 = -138233386.34684;

        pixel_x += c1; pixel_y += c2;
        x = a1*pixel_x+b1*pixel_y;
        y = a2*pixel_x+b2*pixel_y;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "manipulateglobalpath");

	std::stringstream pathread_path;
	if (!is_kcity) {
		pathread_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
	}
	else if (is_kcity && is_pre) {
		pathread_path << ros::package::getPath("slam") << "/config/KCity/KCity_pre_global_path.txt";
	}
	else if (is_kcity && !is_pre) {
		pathread_path << ros::package::getPath("slam") << "/config/KCity/KCity_global_path.txt";
	}
	else  {
		ROS_INFO("INVALID PATHSTREAM!!!!!!!!!!!!!!!");
	}

	std::stringstream pathwrite_path;
	if (!is_kcity) {
		pathwrite_path << ros::package::getPath("slam") << "/FMTC_global_path.txt";
	}
	else if (is_kcity && is_pre) {
		pathwrite_path << ros::package::getPath("slam") << "/KCity_pre_global_path.txt";
	}
	else if (is_kcity && !is_pre) {
		pathwrite_path << ros::package::getPath("slam") << "/KCity_global_path.txt";
	}
	else  {
		ROS_INFO("INVALID PATHSTREAM!!!!!!!!!!!!!!!");
	}

	// read path
	std::ifstream file;
	file.open(pathread_path.str());
	
	std::string line;
	std::string token;

	std::vector<double> utm_x;
	std::vector<double> utm_y;
	std::vector<int> flag;
	size_t length;

	int linecnt = 0;
	while (std::getline(file, line)) {
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
				default:
				break;
			}
		}
	}
	length = utm_x.size();
	file.close();

	if (utm_x.size() > 0) {
		ROS_INFO("path loaded");
	}
	else {
		ROS_INFO("path load error");
	}
	
	if (is_kcity && !is_pre) {
		int flag_cnt = 0;
		for (int i=0; i<length; i++) {
			if (flag[i] == 1) {
				flag_cnt++;
			}
		}

		for (int i=4*flag_cnt/5+30; i<flag_cnt; i++) {
			flag[i] = 2;
		}

		flag_cnt = 0;
		for (int i=0; i<length; i++) {
			if (flag[i] == 1) {
				flag_cnt++;
			}
		}

		std::vector<double> tmp_x;
		tmp_x.assign(utm_x.begin()+4*flag_cnt/6, utm_x.begin()+flag_cnt);
		utm_x.insert(utm_x.begin()+flag_cnt, tmp_x.begin(), tmp_x.end());

		std::vector<double> tmp_y;
		tmp_y.assign(utm_y.begin()+4*flag_cnt/6, utm_y.begin()+flag_cnt);
		utm_y.insert(utm_y.begin()+flag_cnt, tmp_y.begin(), tmp_y.end());

		std::vector<int> tmp_flag(flag_cnt - 4*flag_cnt/6, 2);
		flag.insert(flag.begin()+flag_cnt, tmp_flag.begin(), tmp_flag.end());

		length = utm_x.size();

		int flag4_start_idx;
		int flag4_end_idx;
		for (int i=1; i<length; i++) {
			if (flag[i] == 4 && flag[i-1] == 3) {
				flag4_start_idx = i;
			}
			else if (flag[i] == 5 && flag[i-1] == 4) {
				flag4_end_idx = i;
			}
		}

		//std::cout<<flag_start_idx+(flag_end_idx-flag_start_idx)/5+30<<std::endl;
		//std::cout<<flag_end_idx<<std::endl;

		std::vector<double> sub_path_x;
		sub_path_x.assign(utm_x.begin()+flag4_start_idx+(flag4_end_idx-flag4_start_idx)/5+30,  utm_x.begin()+flag4_end_idx);

		std::vector<double> sub_path_y;
		sub_path_y.assign(utm_y.begin()+flag4_start_idx+(flag4_end_idx-flag4_start_idx)/5+30,  utm_y.begin()+flag4_end_idx);

		for (int i=0; i<sub_path_x.size(); i++) {
			double pixel_x, pixel_y;
			XYToPixel(pixel_x, pixel_y, sub_path_x[i], sub_path_y[i], is_kcity);
			PixelToXY(sub_path_x[i], sub_path_y[i], pixel_x - 110, pixel_y, is_kcity);
		}

		utm_x.insert(utm_x.begin(), sub_path_x.begin(), sub_path_x.end());
		utm_y.insert(utm_y.begin(), sub_path_y.begin(), sub_path_y.end());

		std::vector<double> sub_path_flag(flag4_end_idx - flag4_start_idx - (flag4_end_idx-flag4_start_idx)/5 - 30, 0);
		flag.insert(flag.begin(), sub_path_flag.begin(), sub_path_flag.end());

		length = utm_x.size();

		int flag3_start_idx, flag3_end_idx;
		for (int i=1; i<length; i++) {
			if (flag[i] == 3 && flag[i-1] == 2) {
				flag3_start_idx = i;
			}
			else if (flag[i] == 4 && flag[i-1] == 3) {
				flag3_end_idx = i;
			}
		}

		for (int i=flag3_start_idx; i<flag3_start_idx+60; i++) {
			flag[i] = 2;
		}

		for (int i=1; i<length; i++) {
			if (flag[i] == 4 && flag[i-1] == 3) {
				flag4_start_idx = i;
			}
			else if (flag[i] == 5 && flag[i-1] == 4) {
				flag4_end_idx = i;
			}
		}
		for (int i=flag4_start_idx; i<flag4_start_idx+50; i++) {
			flag[i] = 3;
		}

		int flag5_start_idx, flag5_end_idx;
		for (int i=1; i<length; i++) {
			if (flag[i] == 5 && flag[i-1] == 4) {
				flag5_start_idx = i;
			}
			else if (flag[i] == 6 && flag[i-1] == 5) {
				flag5_end_idx = i;
			}
		}

		for (int i=flag5_start_idx; i<flag5_start_idx+50; i++) {
			flag[i] = 4;
		}
	}
	else if (is_kcity && is_pre) {
		int flag6_start_idx, flag6_end_idx;
		for (int i=1; i<length; i++) {
			if (flag[i] == 6 && flag[i-1] == 5) {
				flag6_start_idx = i;
			}
			else if (flag[i] == 7 && flag[i-1] == 6) {
				flag6_end_idx = i;
			}
		}
		
		for (int i=flag6_end_idx-70; i<flag6_end_idx; i++) {
			flag[i] = 7;
		}

		int flag3_start_idx, flag3_end_idx;
		for (int i=1; i<length; i++) {
			if (flag[i] == 3 && flag[i-1] == 2) {
				flag3_start_idx = i;
			}
			else if (flag[i] == 4 && flag[i-1] == 3) {
				flag3_end_idx = i;
			}
		}

		for (int i=flag3_start_idx; i<flag3_start_idx+60; i++) {
			flag[i] = 2;
		}
	}

	std::ofstream write_file(pathwrite_path.str());
	if (write_file.is_open()) {
		for (int i = 0; i < length; i++) {
			write_file << std::fixed << std::setprecision(6) << utm_x[i] <<" "<< utm_y[i] <<" "<< flag[i] <<std::endl;
		}
		write_file.close();
	}

	return 0;
}
 
