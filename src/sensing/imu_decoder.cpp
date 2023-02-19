#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include "slam/Float_header.h"
#include "std_msgs/Float64.h"
#include "slam/VehicleState.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "slam/GlobalPathPoint.h"
#include "slam/Imu.h"
#include "slam/Gps.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "ros/time.h"

#include "UnixtimeToSec.h"
#include "UTM.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<slam::Float_header, slam::Float_header> MSP;

class IMU_Decoder{
    public:

    IMU_Decoder(){
	    ros::param::getCached("/angle_offset", angle_offset);
    	pub_ = data_.advertise<slam::Imu>("imu", 2);
	//pub_angle_ = data_.advertise<slam::Imu>("djlee", 2);
	    sub_data_ = data_.subscribe("/imu/data", 2, &IMU_Decoder::callback_data, this);
	    g_sub_ = data_.subscribe("/gps", 2, &IMU_Decoder::g_callback, this);
	    v_sub_ = data_.subscribe("/vehicle_state", 2, &IMU_Decoder::v_callback, this);
    	// sub_ang_diff_ = nh_.subscribe("/angle_diff", 2, &IMU_Decoder::callback, this);
	//sub_mag_ = mag_.subscribe("/imu/mag", 2, &IMU_Decoder::callback_mag, this);
        ros::param::get("/is_kcity", is_kcity);
	ros::param::get("/is_delivery", is_delivery);
        if(!is_kcity && !is_delivery) path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
	else if(!is_kcity && is_delivery) path_stream << ros::package::getPath("slam") << "/config/FMTC/delivery.txt";
        else path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_global_path.txt";
        load_global_path();
    }

    void load_global_path(){
        string in_line;
        ifstream in(path_stream.str());
        while(getline(in, in_line)){
            stringstream ss(in_line);
            string token;
            vector<string> result;
            while(getline(ss, token, delimiter_)){
                result.push_back(token);
            }
            slam::GlobalPathPoint point;
	    	if (result.size() == 0) break;
			else if(result.at(0) == "//") continue;
            point.x = stod(result.at(0));
            point.y = stod(result.at(1));
            global_path_.push_back(point);
        }
    }

    void callback_data(const sensor_msgs::Imu::ConstPtr& msg){
        slam::Imu rt;
        rt.header = msg->header;
        geometry_msgs::Quaternion q = msg->orientation;
        rt.theta = std::atan2( 2*(q.x*q.y+q.z*q.w), 1-2*(q.y*q.y+q.z*q.z) ); // - angle_offset*M_PI/180;
        rt.theta = remainder(theta_, 2*M_PI); // (rt.theta,2*M_PI);
        double roll = std::atan2( 2*(q.x*q.w+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y) );
        double pitch = asin(2*(q.y*q.w-q.x*q.z));
        rt.local_ax = std::cos(pitch)*msg->linear_acceleration.x + std::sin(roll)*std::sin(pitch)*msg->linear_acceleration.y + std::cos(roll)*std::sin(pitch)*msg->linear_acceleration.z;
        rt.local_ay = std::cos(roll)*msg->linear_acceleration.y - std::sin(roll)*msg->linear_acceleration.z;
        rt.omega = -std::sin(pitch)*msg->angular_velocity.x + std::sin(roll)*std::cos(pitch)*msg->angular_velocity.y + std::cos(roll)*std::cos(pitch)*msg->angular_velocity.z;
        //rt.err_a = std::abs(msg->linear_acceleration.z - 9.79945);
        //rt.err_omega = sqrt( msg->angular_velocity.x*msg->angular_velocity.x + msg->angular_velocity.y*msg->angular_velocity.y );
        rt.angle_diff = angle_offset;
        pub_.publish(rt);
    }

    /*void callback_mag(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    	   
        double theta_mag = std::atan2(msg->vector.y,msg->vector.x);
        theta_ = M_PI/2 - (theta_mag + theta_diff_);
    }*/
    
    void v_callback(const slam::VehicleState msg){
        gear_state = msg.gear;
    }

    void g_callback(const slam::Gps msg){
    	pair<double, double> xy = std::make_pair(msg.x, msg.y);

        if(is_initial){
            calculate_initial_heading(xy.first,xy.second);
            is_initial=false;
        }
        
        if(sqrt(pow(xy.first - prev_xy.first,2)+pow(xy.second - prev_xy.second,2)) > 1){
            prev_xy = xy;
            return;
        }
        
        if(sqrt(pow(xy.first - prev_xy.first,2)+pow(xy.second - prev_xy.second,2)) > 0.3){
            double temp;		
            theta_ = atan2(xy.second - prev_xy.second, xy.first - prev_xy.first);
            // if(abs(temp - theta_) < 0.4) theta_ = temp;		
            prev_xy = xy;
        }
    }

    void calculate_initial_heading(double x, double y){
        double minimum_distance=INFINITY;
        auto minimum_point=global_path_.begin();
        for(auto it=global_path_.begin();it!=global_path_.end();it++){
            double distance=sqrt(pow(x-it->x,2)+pow(y-it->y,2));
            if(distance<minimum_distance){
                minimum_distance=distance;
                minimum_point=it;
            }
        }

        auto calculate_point=minimum_point;
        for(int i=0;i<5;i++){
            if(calculate_point==global_path_.end()){
                return;
            }
            calculate_point++;
        }
        
        theta_=atan2((minimum_point+5)->y-minimum_point->y,(minimum_point+5)->x-minimum_point->x);
    }

    private:
    ros::NodeHandle data_;	
    ros::NodeHandle mag_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_angle_;
    ros::Subscriber sub_ang_diff_;
    ros::Subscriber sub_data_;
    ros::Subscriber sub_mag_;
    ros::Subscriber h_sub_;
    ros::Subscriber g_sub_;
    ros::Subscriber v_sub_;
    pair<double, double> prev_xy = std::make_pair(0,0);
    double theta_{-2.1};
    double theta_diff_ = -8.60/180*M_PI;  //difference between magnetic north and truth north
    double angle_offset = 0;
    int gear_state = 0;
    stringstream path_stream;
    vector<slam::GlobalPathPoint> global_path_; 
    const char delimiter_ = ' '; 
    bool is_kcity;
    bool is_delivery;
    bool is_initial=true;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_decoder");
    IMU_Decoder IMUObject;

    ros::spin();
}
