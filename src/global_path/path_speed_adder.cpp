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
bool is_kcity = false;
bool is_delivery = true;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathspeedadder");

	std::stringstream pathread_path;
	if (!is_kcity && !is_delivery) {
		pathread_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
	}
	else if (!is_kcity && is_delivery) {
		pathread_path << ros::package::getPath("slam") << "/config/FMTC/delivery3.txt";
	}
	else if (is_kcity) {
		pathread_path << ros::package::getPath("slam") << "/config/KCity/KCity_global_path.txt";
	}
	else  {
		ROS_INFO("INVALID PATHSTREAM!!!!!!!!!!!!!!!");
	}

	std::stringstream pathwrite_path;
	if (!is_kcity && !is_delivery) {
		pathwrite_path << ros::package::getPath("slam") << "/FMTC_global_path.txt";
	}
	else if (!is_kcity && is_delivery) {
		pathwrite_path << ros::package::getPath("slam") << "/delivery.txt";
	}
	else if (is_kcity) {
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
					break;
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

	std::vector<int> flag_start_idx(500, 0);
	int cur_flag = -1;
	for (int i=0; i<length; i++) {
		if (cur_flag != flag[i]) {
			cur_flag = flag[i];
			flag_start_idx[flag[i]] = i;
		}
	}
	std::vector<double> speed(length, 0);
	std::vector<int> curve(length, 0);

	if (!is_kcity && !is_delivery) {
		for(int i=0; i<length; i++) {
			// flag 0: obstacle static && traffic light, 2.5m/s default
			if (i>=flag_start_idx[0] && i<flag_start_idx[1]) {
				speed[i] = 2;
			}
			// flag 1: obstacle static, 2.5m/s default
			else if (i>=flag_start_idx[1] && i<flag_start_idx[2]) {
				speed[i] = 2;
			}
			// flag 2: crosswalk, 3830~4220 -> 5~3m/s, 4220~4790 -> 3m/s, 5900~end -> 5~3m/s , default 5m/s
			else if (i>=flag_start_idx[2] && i<3830) {
				speed[i] = 5;			
			}
			else if (i>=3830 && i<4220) {
				speed[i] = 5 - 1.5*static_cast<double>(i-3830)/static_cast<double>(4220-3830);
			}
			else if (i>=4220 && i<4790) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=4790 && i<6000) {
				speed[i] = 5;
			}
			else if (i>=6000 && i<6200) {
				speed[i] = 5 - 1.5*static_cast<double>(i-6000)/static_cast<double>(6200-6000);
			}
			else if (i>=6200 && i<flag_start_idx[3]) {
				speed[i] = 3.5;
			}
			// flag 3: traffic light, 3m/s default
			else if (i>=flag_start_idx[3] && i<flag_start_idx[4]) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			// flag 4: delivery A search, start~7000->3m/s, 7250~7500->5~3m/s, 7500~end->3m/s 5m/s default
			else if (i>=flag_start_idx[4] && i<7000) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=7000 && i<7250) {
				speed[i] = 5;
			}
			else if (i>=7250 && i<7500) {
				speed[i] = 5 - 3*static_cast<double>(i-7250)/static_cast<double>(7500-7250);
			}
			else if(i>=7500 && i<flag_start_idx[5]) {
				speed[i] = 2;
			}
			// flag 5: delivery A, default 2.5m/s
			else if (i>=flag_start_idx[5] && i<flag_start_idx[6]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 6: delivery B search, default 3m/s
			else if (i>=flag_start_idx[6] && i<flag_start_idx[7]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 7: delivery B, default 2.5m/s
			else if (i>=flag_start_idx[7] && i<flag_start_idx[8]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 8: driving, start~9750->3m/s, 9900~10100 -> 5~3m/s, 10100~end -> 3m/s, default 5m/s
			else if (i>=flag_start_idx[8] && i<9750) {
				speed[i] = 3;
				curve[i] = 1;
			}
			else if (i>=9750 && i<9900) {
				speed[i] = 5;
			}
			else if (i>=9900 && i<10100) {
				speed[i] = 5 - 1.5*static_cast<double>(i-9800)/static_cast<double>(10100-9800);
			}
			else if (i>=10100 && i<10300) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=10300 && i<10490) {
				speed[i] = 3.5;
			}
			else if (i>=10490 && i<flag_start_idx[9]) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			// flag 9: parking search, start~11050 -> 3m/s, default 2m/s
			else if (i>=flag_start_idx[9] && i<11050) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=11050 && i<flag_start_idx[10]) {
				speed[i] = 2;
			}
			//flag 10: finish, default 5m/s
			else if (i>=flag_start_idx[10] && i<flag_start_idx[102]) {
				speed[i] = 5;
			}
			// flag 102~104: parking 1, 1.5m/s default
			else if (i>=flag_start_idx[102] && i<length) {
				speed[i] = 1.5;
				curve[i] = 1;
			}
			else {}
		}
	}
	else if (is_kcity) {
		for(int i=0; i<length; i++) {
			// flag 1: parking search, 200~400->5~2m/s, 400~end->2m/s, 4m/s default
			if (i>=flag_start_idx[1] && i<200) {
				speed[i] = 5;
			}
			else if (i>=200 && i<400) {
				speed[i] = 5 - 3*static_cast<double>(i-200)/static_cast<double>(400-200);
			}
			else if(i>=400 && i<flag_start_idx[2]) {
				speed[i] = 2;
			}
			// flag 2: intersection straight, start~900->3m/s, 1100~1300->5~3m/s, 1300~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[2] && i<900) {
				speed[i] = 3;
				curve[i] = 1;
			}
			else if (i>=900 && i<1100) {
				speed[i] = 5;
			}
			else if (i>=1100 && i<1300) {
				speed[i] = 5 - 2*static_cast<double>(i-1100)/static_cast<double>(1300-1100);
			}
			else if(i>=1300 && i<flag_start_idx[3]) {
				speed[i] = 3;
			}
			// flag 3: intersection straight, 1600~1800->5~3m/s, 1800~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[3] && i<1600) {
				speed[i] = 5;
			}
			else if (i>=1600 && i<1800) {
				speed[i] = 5 - 2*static_cast<double>(i-1600)/static_cast<double>(1800-1600);
			}
			else if (i>=1800 && i<flag_start_idx[4]) {
				speed[i] = 3;
			}
			// flag 4: obstacle static && intersection straight, start~2050->3.5m/s, 2050~2250->3.5~2.5m/s, 2.5m/s default
			else if (i>=flag_start_idx[4] && i<2050) {
				speed[i] = 3.5;
			}
			else if (i>=2050 && i<2250) {
				speed[i] = 3.5 - 1.5*static_cast<double>(i-2050)/static_cast<double>(2250-2050);
			}
			else if (i>=2250 && i<flag_start_idx[5]) {
				speed[i] = 2;
			}
			// flag 5: before delivery, 3m/s default
			else if (i>=flag_start_idx[5] && i<flag_start_idx[6]) {
				speed[i] = 3;
			}
			// flag 6: delivery A search, 2.5m/s default
			else if (i>=flag_start_idx[6] && i<flag_start_idx[7]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 7: delivery A, 2.5m/s default 
			else if (i>=flag_start_idx[7] && i<flag_start_idx[8]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 8: driving, start~5150->3m/s, 5350~end->5~3m/s, 5m/s default
			else if (i>=flag_start_idx[8] && i<5080) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=5080 && i<5280) {
				speed[i] = 5;
			}
			else if (i>=5280 && i<flag_start_idx[9]) {
				speed[i] = 5 - 1.5*static_cast<double>(i-5280)/static_cast<double>(flag_start_idx[9]-5280);
			}
			// flag 9: driving, start~6030->3m/s, 6600~6800->5~3m/s, 6800~end->3m/s, default 5m/s
			else if (i>=flag_start_idx[9] && i<5530) {
				speed[i] = 3.5;
			}
			else if (i>=5530 && i<5700) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=5700 && i<5840) {
				speed[i] = 3.5;
			}
			else if (i>=5840 && i<5960) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=5960 && i<6530) {
				speed[i] = 5;
			}
			else if (i>=6530 && i<6730) {
				speed[i] = 5 - 1.5*static_cast<double>(i-6530)/static_cast<double>(6730-6530);
			}
			else if (i>=6730 && i<6870) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=6870 && i<7060) {
				speed[i] = 3.5;
			}
			else if (i>=7060 && i<flag_start_idx[10]) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			// flag 10: driving, 3m/s default
 			else if (i>=flag_start_idx[10] && i<flag_start_idx[11]) {
				speed[i] = 3.5;
			}
			// flag 11: delivery B search, 2.5m/s default
			else if (i>=flag_start_idx[11] && i<flag_start_idx[12]) {
				speed[i] = 2;
			}
			// flag 12: delivery B, 2.5m/s default
			else if (i>=flag_start_idx[12] && i<flag_start_idx[13]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 13: intersection straight, 3m/s default
			else if (i>=flag_start_idx[13] && i<flag_start_idx[14]) {
				speed[i] = 3;
				curve[i] = 1;
			}
			// flag 14: intersection left unprotected, 9800~10000->5~3m/s, 10000~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[14] && i<9730) {
				speed[i] = 5;
			}
			else if (i>=9730 && i<9930) {
				speed[i] = 5 - 2*static_cast<double>(i-9730)/static_cast<double>(9930-9730);
			}
			else if(i>=9930 && i<flag_start_idx[15]) {
				speed[i] = 3;
			}
			// flag 15: intersection left, start~10800->3m/s, 11300~11500->5~3m/s, 11500~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[15] && i<10180) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=10180 && i<10550) {
				speed[i] = 3.5;
			}
			else if (i>=10550 && i<10730) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=10730 && i<11230) {
				speed[i] = 5;
			}
			else if (i>=11230 && i<11430) {
				speed[i] = 5 - 2*static_cast<double>(i-11230)/static_cast<double>(11430-11230);
			}
			else if (i>=11430 && i<flag_start_idx[16]) {
				speed[i] = 3;
			}
			// flag 16: intersection straight, start~12220->3m/s, 12700~12900->5~3m/s, 12900~end->3m/s
			else if (i>=flag_start_idx[16] && i<12150) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=12150 && i<12630) {
				speed[i] = 5;
			}
			else if (i>=12630 && i<12830) {
				speed[i] = 5 - 2*static_cast<double>(i-12630)/static_cast<double>(12830-12630);
			}
			else if (i>=12830 && i<flag_start_idx[17]) {
				speed[i] = 3;
			}
			// flag 17: intersection straight, 13800~14000->5~3m/s, 14000~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[17] && i<13730) {
				speed[i] = 5;
			}
			else if (i>=13730 && i<13930) {
				speed[i] = 5 - 2*static_cast<double>(i-13730)/static_cast<double>(13930-13730);
			}
			else if(i>=13930 && i<flag_start_idx[18]) {
				speed[i] = 3;
			}
			// flag 18: intersection straight, 14200~14400->5~3m/s, 14400~end->3m/s, 5m/s default
			else if (i>=flag_start_idx[18] && i<14130) {
				speed[i] = 5;
			}
			else if (i>=14130 && i<14330) {
				speed[i] = 5 - 2*static_cast<double>(i-14130)/static_cast<double>(14330-14130);
			}
			else if (i>=14330 && i<flag_start_idx[19]) {
				speed[i] = 3;
			}
			// flag 19: intersection straight unsigned, 5.4m/s default
			else if (i>=flag_start_idx[19] && i<flag_start_idx[20]) {
				speed[i] = 5.4;
			}
			// flag 20: finish, 5.4m/s default
			else if (i>=flag_start_idx[20] && i<flag_start_idx[101]) {
				speed[i] = 5.4;
			}
			// flag 101~106: parking, 2m/s default
			else if (i>=flag_start_idx[101] && i<flag_start_idx[300]) {
				speed[i] = 1.5;
				curve[i] = 1;
			}
			// flag 300: obstacle static subpath
			else if (i>=flag_start_idx[300] && i<length) {
				speed[i] = 2;
			}
		}
	}
	else if (!is_kcity && is_delivery) {
		for(int i=0; i<length; i++) {
			// flag 6: delivery A search, 2.5m/s default
			if (i>=flag_start_idx[6] && i<flag_start_idx[7]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 7: delivery A, 2.5m/s default 
			else if (i>=flag_start_idx[7] && i<flag_start_idx[8]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 8: driving, start~5150->3m/s, 5350~end->5~3m/s, 5m/s default
			else if (i>=flag_start_idx[8] && i<900) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=900 && i<1050) {
				speed[i] = 5;
			}
			else if (i>=1050 && i<flag_start_idx[9]) {
				speed[i] = 5 - 1.5*static_cast<double>(i-1050)/static_cast<double>(flag_start_idx[9]-1050);
			}
			// flag 9: driving, start~6030->3m/s, 6600~6800->5~3m/s, 6800~end->3m/s, default 5m/s
			else if (i>=flag_start_idx[9] && i<1690) {
				speed[i] = 3.5;
			}
			else if (i>=1690 && i<1870) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=1870 && i<2070) {
				speed[i] = 3.5;
			}
			else if (i>=2070 && i<2190) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=2190 && i<2880) {
				speed[i] = 5;
			}
			else if (i>=2880 && i<3080) {
				speed[i] = 5 - 1.5*static_cast<double>(i-2880)/static_cast<double>(3080-2880);
			}
			else if (i>=3080 && i<3240) {
				speed[i] = 3.5;
				curve[i] = 1;
			}
			else if (i>=3240 && i<3530) {
				speed[i] = 3.5 - 1.5 * static_cast<double>(i-3240)/static_cast<double>(3530-3240);
			}
			else if (i>=3530 && i<flag_start_idx[10]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 10: driving, 3m/s default
 			else if (i>=flag_start_idx[10] && i<flag_start_idx[11]) {
				speed[i] = 2;
			}
			// flag 11: delivery B search, 2.5m/s default
			else if (i>=flag_start_idx[11] && i<flag_start_idx[12]) {
				speed[i] = 2;
			}
			// flag 12: delivery B, 2.5m/s default
			else if (i>=flag_start_idx[12] && i<flag_start_idx[13]) {
				speed[i] = 2;
				curve[i] = 1;
			}
			// flag 13: intersection straight, 3m/s default
			else if (i>=flag_start_idx[13] && i<flag_start_idx[300]) {
				speed[i] = 3;
				curve[i] = 1;
			}
			// flag 300: obstacle static subpath
			else if (i>=flag_start_idx[300] && i<length) {
				speed[i] = 2;
			}
		}
	}

	std::ofstream write_file(pathwrite_path.str());
	if (write_file.is_open()) {
		for (int i = 0; i < length; i++) {
			write_file << std::setprecision(10) << utm_x[i] <<" "<< utm_y[i] <<" "<< flag[i] <<" "<< speed[i] <<" "<< curve[i] <<std::endl;
		}
		write_file.close();
	}

	return 0;
}
 
