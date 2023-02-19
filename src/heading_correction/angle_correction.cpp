#include "ros/ros.h"
#include "ros/package.h"

#include "std_msgs/Float64.h"
#include "slam/Float_header.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

using namespace cv;
using namespace std;

class Angle_correction{

	private:
    	ros::NodeHandle nh;
    	ros::Publisher pub;
	ros::Publisher pub_;
	ros::Subscriber sub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub;	
	double theta_average{0};
	double theta_costmap{0};
	//double diffAngle_{0};

    	
	public:
	int lane_state{0};
    	Angle_correction()
	: it_(nh)
	{
        pub = nh.advertise<slam::Float_header>("/angle_2", 2);
		pub_ = nh.advertise<slam::Float_header>("/angle_1", 2);
		sub = nh.subscribe<sensor_msgs::Image>("/lane_costmap", 2, &Angle_correction::calAngle2, this);
                image_sub = it_.subscribe("/lane_image", 2, &Angle_correction::calAngle, this);
     		
    	}

	void calAngle(const sensor_msgs::ImageConstPtr& msg)
	{
		//slam::Data rt;

		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		Mat img_canny;		
		Canny(cv_ptr->image, img_canny, 150, 255);
		vector<Vec4i> linesP;
		//HoughLines(src, dst, rho, theta, threshold, min_line, max_line_gao)
		HoughLinesP(img_canny, linesP, 1, (CV_PI / 180), 50, 50, 10);
		Mat img_houghP;
		cv_ptr->image.copyTo(img_houghP);
		Mat img_lane;
		threshold(img_canny, img_lane, 200, 255, THRESH_MASK);
		Point p1, p2;
		double SUM = 0;
		double num = 0;
		for (size_t i = 0; i < linesP.size(); i++)
		{
			Vec4i l = linesP[i];
			line(img_houghP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, 8);	
			line(img_lane, Point(l[0], l[1]), Point(l[2], l[3]), Scalar::all(255), 1, 8);
			p1=Point(l[0], l[1]);
    			p2=Point(l[2], l[3]);
    			double angle = atan2(p1.x - p2.x, p1.y - p2.y);
			double angleinDegree = angle * 180 / 3.141592;
			if (angleinDegree < -90)		
				angleinDegree += 180.0;
			else if (angleinDegree > 90)		
				angleinDegree -= 180.0;

			// cout << " theta_degree!!!!!!!!!!!!!!: " << angleinDegree << endl;
	
			SUM += angleinDegree;
			num++;
			
		}

		slam::Float_header rt;
		theta_average = (SUM / num) ;
		//return theta_average;
		//pub.publish(rt);
		rt.data = theta_average;
		rt.header = msg->header;
		pub_.publish(rt);
		ROS_INFO("angle correction working!!!!!!!!!!!!!: %f", theta_average );
	}

	void calAngle2(const sensor_msgs::Image::ConstPtr& img)
	{
		//slam::Data rt;
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
		Mat img_canny;		
		Canny(cv_ptr->image, img_canny, 150, 255);
		vector<Vec4i> linesP;
		//HoughLines(src, dst, rho, theta, threshold, min_line, max_line_gao)
		HoughLinesP(img_canny, linesP, 1, (CV_PI / 180), 50, 50, 10);
		Mat img_houghP;
		cv_ptr->image.copyTo(img_houghP);
		Mat img_lane;
		threshold(img_canny, img_lane, 200, 255, THRESH_MASK);
		Point p1, p2;
		double SUM = 0;
		double num = 0;
		for (size_t i = 0; i < linesP.size(); i++)
		{
			Vec4i l = linesP[i];
			line(img_houghP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, 8);	
			line(img_lane, Point(l[0], l[1]), Point(l[2], l[3]), Scalar::all(255), 1, 8);
			p1=Point(l[0], l[1]);
    			p2=Point(l[2], l[3]);
    			double angle = atan2(p1.x - p2.x, p1.y - p2.y);
			double angleinDegree = angle * 180 / 3.141592;
			if (angleinDegree < -90)		
				angleinDegree += 180.0;
			else if (angleinDegree > 90)		
				angleinDegree -= 180.0;

			cout << " theta_degree______: " << angleinDegree << endl;
	
			SUM += angleinDegree;
			num++;
			
		}

		slam::Float_header rt;
		theta_costmap = (SUM / num) ;
		//return theta_costmap;
		//pub.publish(rt);
		rt.data = theta_costmap;
		rt.header = img->header;
		pub.publish(rt);
		ROS_INFO("angle correction working____________: %f", theta_costmap );
	}
	/*
	void diffAngle()
	{
		diffAngle_ = theta_costmap - theta_average ; 
		slam::Data rt;
		rt.angle_diff = diffAngle_;
		pub.publish(rt);
	}
	*/
};

int main(int argc, char **argv){
    	ros::init(argc, argv, "angle_correction");
	Angle_correction angle_correction;
    	ros::spin();
}
