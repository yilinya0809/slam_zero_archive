#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt32.h"
#include <iostream>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include <Eigen/Dense>
#include "opencv2/core/eigen.hpp"

using namespace cv;
using namespace std;


class Lane_image_publisher{
   private:
	ros::NodeHandle nh;
        ros::Publisher pub;
	ros::Subscriber sub;
   public:
	Eigen::Matrix3d cameraMatrix;
	Eigen::Matrix3d trans;
        Eigen::Matrix3d T_i2g;
        Eigen::Matrix3d translation;
        Mat trans_cv;
        Mat camera_cv;
	
	float f_u = cameraMatrix(0, 0);
        float f_v = cameraMatrix(1, 1);
        float c_u = cameraMatrix(0, 2);
        float c_v = cameraMatrix(1, 2);
        float c1 = cos(alpha);
        float s1 = sin(alpha);

        

        double h = 89.7/2;
        double alpha = -0.02;

	Lane_image_publisher() {
		pub = nh.advertise<sensor_msgs::Image>("/lane_image", 2);
		sub = nh.subscribe("/camera_front", 2, &Lane_image_publisher::image_callback, this);

		cameraMatrix << 0.5011e03, 0.0, 3.20e02,
				0.0, 0.5011e03, 2.40e02,
				0.0, 0.0, 1.0;
				
		 T_i2g << (-1 / f_u), 0, (c_u / f_u), 
		0, (s1 * c1 / f_v), (- c_v * s1 / f_v - c1), 
		0, (-c1 / h / f_v), (c_v * c1 / h / f_v - s1 / h);
		translation << 1, 0, 250. , 0, -1, 1000., 0, 0, 1;
		
		T_i2g *= h;

		trans = translation*T_i2g;

		eigen2cv(cameraMatrix, camera_cv);
		eigen2cv(trans, trans_cv);

	}

	void image_callback(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg);
		Mat img_gray;		
		cvtColor(cv_ptr->image, img_gray, CV_RGB2GRAY);
		imshow("1", img_gray);
		waitKey(1);
		Size warpsize(500,1000);

		Mat img_blur;
		GaussianBlur(img_gray, img_blur, Size(5,5), 0);
		       
		Mat img_canny;
		Canny(img_blur, img_canny, 150, 255);
	       
		vector<Vec4i> linesP;
		HoughLinesP(img_canny, linesP, 1, (CV_PI / 180), 50, 50, 10);
		
		for (size_t i = 0; i < linesP.size(); i++)
		{
		     Vec4i l = linesP[i];
		     line(img_canny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar::all(255), 1, 8);
		}
		
		/*
		vector<Vec4i> lane_lines;
		for (size_t i = 0; i < linesP.size(); i++)
		{
		     Vec4i l = linesP[i];
		     p1 = Point(l[0], l[1]);
		     p2 = Point(l[2], l[3]);
		     double angle = atan2(p1.x - p2.x , p1.y - p2.y);
		     double angleinDegree = angle * 180 / 3.141592 ;
		     if (angleinDegree < -90)
			angleinDegree += 180.0;
		     else if (angleinDegree > 90)
			angleinDegree -= 180.0;

		     if (angleinDegree < 40 || angleinDegree > -40){
		        lane_lines.push_back(l);
		     }
		     line(img_canny, Point(lane_lines[0], lane_lines[1]), Point(lane_lines[2], lane_lines[3]), Scalar::all(255), 1, 8);
		}
		*/

		Mat birdview;
		warpPerspective(img_canny, birdview, trans_cv, warpsize);

		Mat img_fin = birdview(Range(500,730), Range(120,360));
		imshow("lane_canny", img_fin);

		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg;
		std_msgs::Header header;
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_fin);
		img_bridge.toImageMsg(img_msg);
		pub.publish(img_msg);
	}
           
};




int main(int argc, char **argv){
        ros::init(argc, argv, "lane_image_publisher");
	ros::Time::init();
	ros::Rate rate(20); //ROS rate at ??Hz
	Lane_image_publisher lane_image_publisher;
	while(ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
}

