#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <fstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <sstream>
#include <vector>
#include "opencv2/opencv.hpp"

#include "geometry_msgs/Point.h"
#include "slam/Cluster.h"
#include "slam/Clusters.h"
#include "slam/Lidar.h"
#include "slam/LidarPoint.h"
#include "slam/Data.h"
#include "slam/ParkingSpot.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

using namespace std;

typedef pair<double, double> pdd;

class ParkingSpotDetector{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub_count;
    ros::Subscriber sub_position;
    ros::Subscriber sub_lidar;
    
    int count_spot;
    vector<slam::ParkingSpot> parking_spot;
    stringstream path_stream;
    bool is_kcity;
    const char delimiter = ' ';
    pdd position;
    double heading;
    int empty_spot;
    int spot_threshold;
    double lidar_threshold;

public:
    ParkingSpotDetector() {
        pub = nh.advertise<std_msgs::Int32>("/parking_spot", 10);
	pub_count = nh.advertise<slam::ParkingSpot>("/parking_count", 10);
        sub_position = nh.subscribe("/filtered_data", 1, &ParkingSpotDetector::callback_position, this);
        sub_lidar = nh.subscribe("/point_cloud_clusters", 1, &ParkingSpotDetector::callback_lidar, this);

	ros::param::get("/is_kcity",is_kcity);
        parking_spot.clear();
        count_spot = 0;
        empty_spot = - 1;
        spot_threshold = 100;
        lidar_threshold = 20.0;
        if(is_kcity) path_stream << ros::package::getPath("slam")<<"/config/KCity/KCity_parking_spot.txt";
        else path_stream << ros::package::getPath("slam")<<"/config/FMTC/FMTC_parking_spot.txt";
        load_parking_spot();
    }

    void load_parking_spot() {
        string in_line;
        ifstream in(path_stream.str());
        while (getline(in, in_line)) {
            stringstream ss(in_line);
            string token;
            vector<string> result;
            while (getline(ss, token, delimiter)) {
                result.push_back(token);
            }
            slam::ParkingSpot spot;
            if (result.size() == 0) break;
            spot.x = stod(result.at(0));
            spot.y = stod(result.at(1));
            spot.theta = stod(result.at(2));
            spot.width = stod(result.at(3));
            spot.height = stod(result.at(4));
            spot.available = true;
	    //spot.checked = false;
	    //ROS_INFO("%lf, %lf, %lf, %lf, %lf",spot.x, spot.y, spot.theta, spot.width, spot.height);
            parking_spot.push_back(spot);
            count_spot++;
        }
    }

    void callback_position(const slam::Data::ConstPtr& msg) {
        position = {msg->x, msg->y};
        heading = msg->theta;
    }

    void callback_lidar(const slam::Clusters::ConstPtr& msg) {      // GJCheon the great master !!
        std_msgs::Int32 rt;

        for (slam::ParkingSpot& spot : parking_spot) {
            spot.count = 0;
        }
        // (temporary) make car go into parking spot 2
        /////////////////////////////////////
        // spot.count: the number of lidar points in the particular parking spot
        for (slam::Cluster cluster : msg->clusters) {
            for (slam::LidarPoint point : cluster.points) {
                double x = position.first + point.point_2d.x * cos(heading) - point.point_2d.y * sin(heading);
                double y = position.second + point.point_2d.x * sin(heading) + point.point_2d.y * cos(heading);
                for (slam::ParkingSpot &spot : parking_spot) {
                    double horizontal_distance = abs((x - spot.x) * cos(spot.theta) + (y - spot.y) * sin(spot.theta));
                    double vertical_distance = abs((x - spot.x) * sin(spot.theta) - (y - spot.y) * cos(spot.theta));
                    if((horizontal_distance < spot.width / 2) && (vertical_distance < spot.height / 2)) spot.count++;
                }
            }
        }
        // check if the spot is available
        int num = 0;
        std::cout << "[spot.count]" << std::endl;
        for (slam::ParkingSpot& spot : parking_spot) {
            std::cout << "parking_spot_" << num << ".count: " << spot.count << std::endl;
            num++;
            if (!spot.available) continue;
            if (spot.count > spot_threshold) {
                spot.available = false;
                ROS_INFO("spot is not available");
            }
        }
        // pick an empty spot which is not too far from my current position
        for (int index = 0; index < count_spot; index++) {
            slam::ParkingSpot spot = parking_spot.at(index);
            if (!spot.available) continue;
            if (sqrt(pow(position.first - spot.x, 2) + pow(position.second - spot.y, 2)) > lidar_threshold - sqrt(pow(spot.width / 2, 2) + pow(spot.height / 2, 2))) continue;
            ROS_INFO("%d th spot is in the lidar zone",index);
            empty_spot = index;
            rt.data = empty_spot;
            pub.publish(rt);
	    pub_count.publish(parking_spot.at(1));
            return;
        }
        rt.data = -1;
        pub.publish(rt);
	    pub_count.publish(parking_spot.at(1));
    }

    /*void callback_lidar(const slam::Clusters::ConstPtr& msg) {      //Ahnsy
        empty_spot = -1;
        std_msgs::Int32 rt;

        for (slam::ParkingSpot& spot : parking_spot) {
            spot.count = 0;
        }
        // (temporary) make car go into parking spot 2
	    //parking_spot.at(0).count = 200;
        /////////////////////////////////////
        // spot.count: the number of lidar points in the particular parking spot
        for (slam::Cluster cluster : msg->clusters) {
            for (slam::LidarPoint point : cluster.points) {
                double x = position.first + point.point_2d.x * cos(heading) - point.point_2d.y * sin(heading);
                double y = position.second + point.point_2d.x * sin(heading) + point.point_2d.y * cos(heading);
                for (slam::ParkingSpot &spot : parking_spot) {
                    double horizontal_distance = abs((x - spot.x) * cos(spot.theta) + (y - spot.y) * sin(spot.theta));
                    double vertical_distance = abs((x - spot.x) * sin(spot.theta) - (y - spot.y) * cos(spot.theta));
                    if ((horizontal_distance < spot.width / 2) && (vertical_distance < spot.height / 2)) spot.count++;
                }
            }
        }
        // check if the spot is available
        for (slam::ParkingSpot& spot : parking_spot) {
            ROS_INFO("%lf", sqrt(pow(position.first - spot.x, 2) + pow(position.second - spot.y, 2)));
            if (sqrt(pow(position.first - spot.x, 2) + pow(position.second - spot.y, 2)) < lidar_threshold) {
                if (!spot.available) {                  // forbid false positive
                    continue;
                }
                else {                                  // allow false negative
                    if (spot.count > spot_threshold) {
                    spot.available = false;
                    }
                }
            }
        }
        // pick the nearest spot among the available
        for (int index = 0; index < count_spot; index++) {
            slam::ParkingSpot spot = parking_spot.at(index);
            if (spot.available) {
                empty_spot = index;
                break;
            }
        }
        rt.data = empty_spot;
        pub.publish(rt);
    }*/
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "parking_spot_detector");
    ParkingSpotDetector parking_spot_detector;
    ros::spin();
}
