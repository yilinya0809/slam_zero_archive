#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "Eigen/Eigen"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"

using namespace cv;
using namespace std;

#define LEFT 0
#define RIGHT 1

#define BLUE RIGHT
#define YELLOW LEFT

class Conecolor{
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_color;
    ros::Publisher pub_path;
    ros::Subscriber sub_center;
    ros::Subscriber sub_image;
    ros::Subscriber sub_yolo;

    vector<geometry_msgs::Point32> centers;
    
    Eigen::Matrix3d cameraMatrix;

    Eigen::MatrixXd rotationMatrix;

    Eigen::MatrixXd trans;

    cv_bridge::CvImagePtr cone_ptr;
    Mat cone_org;

    Mat distCoeffs;
    Mat camera_cv;

    vector<vector<float>> yolobox;
   
    public:
    Conecolor(): rotationMatrix(3,4), trans(3,4) {
        pub_color = nh.advertise<sensor_msgs::PointCloud>("cone_color", 2);
	pub_path = nh.advertise<nav_msgs::Path>("/cone_path", 2);
	sub_center = nh.subscribe("/cone_center", 2, &Conecolor::callback, this);
	sub_image = nh.subscribe("/camera_front_for_cone", 2, &Conecolor::imgcallback, this);
	sub_yolo = nh.subscribe("/cone_box", 2, &Conecolor::yolocallback, this);

        cameraMatrix << 517, 0, 320,
 			0, 517, 240,
 			0, 0, 1;

        distCoeffs = (Mat1d(1,5) << 0.212, -0.545, 0, 0, 0.351);

        rotationMatrix.row(0) << 0.0377, -0.9992, 0.0141, -0.0979;
	rotationMatrix.row(1) << 0.0455, -0.0124, -0.9989, 0.0655;
	rotationMatrix.row(2) << 0.9983, 0.0383, 0.0450, -0.0098;

        trans = cameraMatrix * rotationMatrix;

        eigen2cv(cameraMatrix, camera_cv);
    }

    void imgcallback(const sensor_msgs::Image::ConstPtr& msg){
	cone_ptr = cv_bridge::toCvCopy(msg); //, sensor_msgs::image_encodings::BGR8);
	cone_org = cone_ptr -> image;
    }

    void yolocallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	yolobox.clear();
	int order = 0;
	vector<float> s;
	for (auto element: msg->data) {
		if (order == 0 || order == 2) {
			s.push_back(element*640);
			order++;
		}
		else if (order == 1 || order == 3) {
			s.push_back(element*480);
			order++;
		}
		else if (order == 4) {
			s.push_back(element);
			order = 0;
			vector<float> cp;
			cp.assign(s.begin(), s.end());
			yolobox.push_back(cp);
			s.clear();
		}
	}
    }

    void callback(const sensor_msgs::PointCloud::ConstPtr& msg){
	centers = msg->points;
        vector<vector<double>> c;
	vector<vector<double>> p;
	
	for (geometry_msgs::Point32 center : centers){
	    vector<double> point;
	    point.push_back(center.z);
	    point.push_back(center.x);
	    point.push_back(center.y);
	    point.push_back(pow(center.x,2)+pow(center.y,2));
	    c.push_back(point);
	}
	
	std::sort(c.begin(), c.end(), [](const vector<double>& a, const vector<double>& b){return a[3] < b[3];});
	std::sort(yolobox.begin(), yolobox.end(), [](const vector<float>& a, const vector<float>& b){return a[2]*a[3] > b[2]*b[3];});
        //trans = cameraMatrix * rotationMatrix;

	for (auto center: c){
	    Eigen::MatrixXd pMat(4,1);
	    pMat(0,0) = center.at(1);
	    pMat(1,0) = center.at(2);
	    pMat(3,0) = 1.0;
	    if(center[1] > 5.5){
		pMat(2,0) = -0.3;
	    } else if(center[1] > 6.5){
	        pMat(2,0) = 0.2;
	    } else{
	        pMat(2,0) = -0.4;
            }
	    //pMat(2,0) = center[1] > 5.5 ? -0.3 : -0.4;
	    
	    Eigen::MatrixXd rMat(3,1);
	    rMat = trans * pMat;
	    int x = rMat(0,0) / rMat(2,0);
	    int y = rMat(1,0) / rMat(2,0);

	    vector<double> i;
	    i.push_back(x); i.push_back(y);
	    p.push_back(i);
	}
	int count = 0;
	/*for (auto cluster_center: p) {
		std::cout<<"cluster center "<<count++<<": "<<cluster_center[0]<<", "<<cluster_center[1]<<std::endl;
	}
	count = 0;
	for (auto onebox: yolobox) {
		std::cout<<"yolobox "<<count++<<": "<<onebox[0]<<", "<<onebox[1]<<", "<<onebox[2]<<", "<<onebox[3]<<", "<<onebox[4]<<std::endl;
	}
	*/
	vector<int> count_yolo;
	for (int n = 0; n < p.size(); n++){
	    vector<double> point = p[n];
	    int check = 0;

	    circle(cone_org, Point(point[0],point[1]), 3, Scalar(255,255,255), -1, 8, 0);
	    if ( c[n][1] < 1.5 ) { 
	    	check = 1;
		if( c.at(n).at(2) < 0){
			c[n][0] = LEFT;
			circle(cone_org, Point(point[0],point[1]), 7, Scalar(255,50,0), -1, 8, 0);
		}
		else{
			c[n][0] = RIGHT;
			circle(cone_org, Point(point[0],point[1]), 7, Scalar(50,255,255), -1, 8, 0);
		}
	    }
	    else{
	    	for (int m = 0; m < yolobox.size(); m++){
			vector<float> box = yolobox[m];
	          	if (fabs(box[0] - point[0])< 1.5*box[2]/(2 - (min(max(1.0, c[n][1]/5.5), 1.3) - 1)) && fabs(box[1] - point[1]) < 1.3*box[3]/(2 - (min(max(1.0, c[n][1]/5.5), 1.3) - 1))){
		    		check = 2;
		    		if(box[4] > 0.5){
					c[n][0] = BLUE;
					circle(cone_org, Point(point[0],point[1]), 7, Scalar(0,0,255), -1, 8, 0);
				}
		    		else{
					c[n][0] = YELLOW;
					circle(cone_org, Point(point[0],point[1]), 7, Scalar(0,255,0), -1, 8, 0);
				}
				yolobox.erase(yolobox.begin()+m);
		    		break;
	    		}
	        }
	    }
	    

	    if (check == 1) count_yolo.push_back(1);
	    else if (check == 2) count_yolo.push_back(2);
	    else count_yolo.push_back(0);
	}
	
	if (!cone_org.empty()) {
		imshow("cone_org", cone_org);
		waitKey(1);
	}

	sensor_msgs::PointCloud rt_center;
	nav_msgs::Path rt_center_;
	vector<geometry_msgs::Point32> rt_centers;
	

	for (int n = 0; n < c.size(); n++){
	    if (count_yolo[n] == 0) continue;
	    if (c[n][0] == 2) continue;

	    vector<double> point = c[n];
	    geometry_msgs::Point32 center;
	    center.x = point.at(1);
	    center.y = point.at(2);
	    center.z = point.at(0);
	    rt_centers.push_back(center);
	}

	for (int n = 0; n < c.size(); n++){
	    if (count_yolo[n] == 0) continue;
	    if (c[n][0] == 2) continue;
	    if (count_yolo[n] == 0) {
		    c[n][0] = 2;
	    }
	    vector<double> point = c[n];
	    geometry_msgs::PoseStamped center_;
	    center_.pose.position.x = point.at(1);
	    center_.pose.position.y = point.at(2);
	    center_.header.seq = point.at(0);
	    rt_center_.poses.push_back(center_);
	}

	rt_center.points = rt_centers;
	pub_color.publish(rt_center);
        pub_path.publish(rt_center_);

    }   
};

int main(int argc, char **argv){
    ros::init(argc, argv, "cone_color");
    Conecolor cone_color;
    ros::spin();
}
