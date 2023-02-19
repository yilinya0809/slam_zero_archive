#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <algorithm>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/OccupancyGrid.h"
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
using namespace cv;


class Conecostmap{
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_map;
    ros::Publisher pub_coor;

    int height_ = 300;
    int width_ = 300;
    cv_bridge::CvImage img_bridge;

    ros::Subscriber sub_color;

    public:
    Conecostmap(){
	pub_map = nh.advertise<sensor_msgs::Image>("cone_costmap", 2);
	//pub_coor = nh.advertise<nav_msgs::Path>("cone_coor", 2);
	
	sub_color = nh.subscribe("cone_color", 2, &Conecostmap::callback, this);
    }
    
    double dist(vector<double> c1, vector<double> c2){
        double distance = (c1.at(2) - c2.at(2))*(c1.at(2) - c2.at(2)) + (c1.at(3)-c2.at(3))*(c1.at(3)-c2.at(3));
        return distance;
    }

    void callback(const sensor_msgs::PointCloud::ConstPtr& msg){
	vector<geometry_msgs::Point32> centers = msg->points;
	vector<vector<double>> c;
	vector<vector<double>> cL;
	vector<vector<double>> cR;

        nav_msgs::Path rnp;
        geometry_msgs::PoseStamped cp;

	for (geometry_msgs::Point32 center : centers){
	    vector<double> point;
	    center.x = center.x * 100 / 3;
	    center.y = - center.y * 100 / 3;
	    point.push_back(center.z);
	    point.push_back(center.x * center.x + center.y * center.y);
	    point.push_back(center.x);
	    point.push_back(center.y);
	    c.push_back(point);
	}
	
	vector<double> pointLi;
	pointLi.push_back(1);
	pointLi.push_back(55*55);
	pointLi.push_back(1);
	pointLi.push_back(-55);
	c.push_back(pointLi);

	vector<double> pointRi;
	pointRi.push_back(0);
	pointRi.push_back(55*55);
	pointRi.push_back(1);
	pointRi.push_back(55);
	c.push_back(pointRi);	

	sort(c.begin(), c.end());
	cout << "ALL" << endl;
	for (vector<double> point : c) {
	    if(point.at(0) == 0) cL.push_back(point);
	    else if (point.at(0) == 1) cR.push_back(point);
	}
	

	for (vector<double> point : c){
	    for (double i : point) cout << i << " ";
	    cout << endl;	
	}
        cout << "CL" << endl;
        for (vector<double> point : cL){
	    for (double i : point) cout << i << " ";
	    cout << endl;	
	}
	cout << "CR" <<endl;
	for (vector<double> point : cR){
	    for (double i : point) cout << i << " ";
	    cout << endl;	
	}
	cout << endl;
	Mat costmap = Mat(height_, width_, CV_8UC1, Scalar(0));

	if (!cL.empty()){
	    //int x1 = (int)(cL.at(0).at(2));
	    //int y1 = (int)(cL.at(0).at(3)) + width_/2;
            //if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) {}
	    //else 
            //    line(costmap, Point(0, y1), Point(x1, y1), Scalar::all(255), 34, 8, 0);
	for (int i = 0; i < (cL.size() - 1); i++) {
	    int x1 = (int)(cL.at(i).at(2));
	    int y1 = (int)(cL.at(i).at(3)) + width_/2;
	    int x2 = (int)(cL.at(i+1).at(2));
	    int y2 = (int)(cL.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(100), 100, 8, 0);
	}for (int i = 0; i < (cL.size() - 1); i++) {
	    int x1 = (int)(cL.at(i).at(2));
	    int y1 = (int)(cL.at(i).at(3)) + width_/2;
	    int x2 = (int)(cL.at(i+1).at(2));
	    int y2 = (int)(cL.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(170), 80, 8, 0);
	}
	}
	if (!cR.empty()){
	    //int x1 = (int)(cR.at(0).at(2));
	    //int y1 = (int)(cR.at(0).at(3)) + width_/2;
            //if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) {}
	    //else 
            //    line(costmap, Point(0, y1), Point(x1, y1), Scalar::all(255), 34, 8, 0);
	for (int i = 0; i < (cR.size() - 1); i++) {
	    int x1 = (int)(cR.at(i).at(2));
	    int y1 = (int)(cR.at(i).at(3)) + width_/2;
	    int x2 = (int)(cR.at(i+1).at(2));
	    int y2 = (int)(cR.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(100), 100, 8, 0);
	}for (int i = 0; i < (cR.size() - 1); i++) {
	    int x1 = (int)(cR.at(i).at(2));
	    int y1 = (int)(cR.at(i).at(3)) + width_/2;
	    int x2 = (int)(cR.at(i+1).at(2));
	    int y2 = (int)(cR.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(170), 80, 8, 0);
	}
	}
	if (!cL.empty()){
	for (int i = 0; i < (cL.size() - 1); i++) {
	    int x1 = (int)(cL.at(i).at(2));
	    int y1 = (int)(cL.at(i).at(3)) + width_/2;
	    int x2 = (int)(cL.at(i+1).at(2));
	    int y2 = (int)(cL.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(255), 60, 8, 0);
	}
	}
	if (!cR.empty()){
	for (int i = 0; i < (cR.size() - 1); i++) {
	    int x1 = (int)(cR.at(i).at(2));
	    int y1 = (int)(cR.at(i).at(3)) + width_/2;
	    int x2 = (int)(cR.at(i+1).at(2));
	    int y2 = (int)(cR.at(i+1).at(3)) + width_/2;
            if (x1<=0 || x1>=height_ || y1<=0 || y1>= width_) continue;
            if (x2<=0 || x2>=height_ || y2<=0 || y2>= width_) continue;
	    line(costmap, Point(x1, y1), Point(x2, y2), Scalar::all(255), 60, 8, 0);
	}
	}

	for (vector<double> point : c) {
	    int x = (int)(point.at(2));
	    int y = (int)(point.at(3)) + width_/2;
            if (x<=0 || x>=height_ || y<=0 || y>= width_) continue;
	    circle(costmap, Point(x,y), 30, Scalar::all(255), -1, 8, 0);
	}
	
	Point2f rotation_center(costmap.cols/2, costmap.rows/2);
        Mat rotation_matrix = getRotationMatrix2D(rotation_center, 90, 1.0);
        warpAffine(costmap, costmap, rotation_matrix, costmap.size());

	imshow("costmap", costmap);
        int key = waitKey(30);

/*
	vector<vector<double>> cn;
	for (vector<double> point : cL) cn.push_back(point);
	for (vector<double> point : cR) cn.push_back(point);

	for (vector<double> point : cn) {
	    cp.pose.position.x = point.at(2);
	    cp.pose.position.y = - point.at(3);
	    cp.pose.position.z = point.at(1);
	    cp.header.seq = point.at(0);
	    rnp.poses.push_back(cp);
	}

	//pub_coor.publish(rnp);
*/
	sensor_msgs::Image rt;
	std_msgs::Header header;
	header.seq = msg -> header.seq;
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, costmap);
        img_bridge.toImageMsg(rt);
        pub_map.publish(rt);
    }
   
};


int main(int argc, char **argv){
    ros::init(argc, argv, "cone_costmap");
    Conecostmap cone_costmap;
    ros::spin();
}
