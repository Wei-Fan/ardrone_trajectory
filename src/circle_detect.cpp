#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "ardrone_trajectory/points.h"

using namespace cv;
using namespace std;

#define DATA_SIZE 600
#define R_DFNT 40
#define Y_DFNT 40

class FindCircles
{
public:
	FindCircles();
	void measurement(double &x, double &y, double u, double v, double pitch, double roll, double h);
	void revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h);
	//void writeData();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;
	ros::Publisher blue_pub;
	ros::Publisher yellow_pub;
	ros::Publisher red_real_pub;
	ros::Publisher red_image_pub;
	//sros::Publisher correct_pub;

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void image_process(Mat img);
	void find_blue(Mat &img, Mat imgThresholded);
	void find_yellow(Mat &img, Mat imgThresholded);
	void find_red(Mat &img, Mat imgThresholded);
	void line_fitting(ardrone_trajectory::points reds, double &k, double &b, double &r, geometry_msgs::Point &foot);

	int data_count = 0;
	double height;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double current_pos[2];
	cv::Point2d ardrone_pos;
	double distant[DATA_SIZE][2];
	double pose[DATA_SIZE][3];
};

FindCircles::FindCircles()
{
	img_sub = node.subscribe("/ardrone/image_raw", 1, &FindCircles::imageCallback, this);//"/ardrone/image_raw" "CamImage"
	altitude_sub = node.subscribe("/ardrone/navdata_altitude", 1, &FindCircles::altitudeCallback, this);
	navdata_sub = node.subscribe("/ardrone/navdata", 1, &FindCircles::navdataCallback, this);
	blue_pub = node.advertise<geometry_msgs::Point>("blue_point", 1000);
	yellow_pub = node.advertise<geometry_msgs::Point>("yellow_point", 1000); //the message of centers is uncertain.
	red_real_pub = node.advertise<ardrone_trajectory::points>("red_real_points", 1000);
	red_image_pub = node.advertise<ardrone_trajectory::points>("red_image_points", 1000);
	//correct_pub = node.advertise<geometry_msgs::Point>("delt", 1000);
	//Mat src_img = imread("/home/wade/catkin_ws/src/find_circles/src/t.jpg", 1);//testing
	//image_process(src_img);//testing 
}

//void FindCircles::writeData()
//{
	/*save for evaluating the error of dsistant*/
//	Mat distantm = Mat(DATA_SIZE, 2, CV_64FC1, distant);
//	Mat posem = Mat(DATA_SIZE, 3, CV_64FC1, pose);
//	ofstream f1("/home/wade/catkin_ws/src/find_circles/test_file/distant.csv");
//	f1 << format(distantm, Formatter::FMT_CSV);
//	ofstream f2("/home/wade/catkin_ws/src/find_circles/test_file/pose.csv");
//	f2 << format(posem, Formatter::FMT_CSV);
//	f1.close();
//	f2.close();
//}

void FindCircles::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0;
	current_pos[0] = -msg.altitude_vision/1000.0 * tan(roll);
	current_pos[1] = msg.altitude_vision/1000.0 * tan(pitch);
	//cout << "position: " << height << '\t' << current_pos[0] << '\t' << current_pos[1] << endl;
}

void FindCircles::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	roll = msg.rotX/180.0*3.1416;
	pitch = msg.rotY/180.0*3.1416;
	yaw = msg.rotZ/180.0*3.1416;
	//cout << "angles: " << roll << '\t' << pitch << '\t' << yaw << endl;
}

void FindCircles::imageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_process(cv_ptr->image);
}

void FindCircles::image_process(Mat img)
{
	ros::Rate loop_rate(10);
	/*evaluation*/
	/*if (data_count == DATA_SIZE)
	{
		writeData();
		exit(0);
	}*/
	Size image_size = img.size();

	/* to get the actual relative position*/
	measurement(ardrone_pos.x, ardrone_pos.y, current_pos[0], current_pos[1], pitch, roll, height);
	ardrone_pos.x = -ardrone_pos.x + image_size.width/2;
	ardrone_pos.y = -ardrone_pos.y + image_size.height/2 - 38;
	//cout << "ardrone_pos in image: " << ardrone_pos.x << '\t' << ardrone_pos.y << endl;
	circle(img, ardrone_pos, 10, Scalar(0, 255, 255), 1, 8);

	/*color abstract*/
	Mat imgThresholded = Mat(image_size, CV_8UC1);
	Mat img_cp = img.clone();
	Mat img_cp2 = img.clone();

	find_blue(img_cp2, imgThresholded);
	find_yellow(img, imgThresholded);
	find_red(img_cp, imgThresholded);
	//imshow("monitor0", img);//testing
	imshow("monitor1", img_cp);//testing
	/*if (char(waitKey(1)) == 'o')
		destroyWindow("monitor0");*/
	if (char(waitKey(1)) == 'o')
		destroyWindow("monitor1");
	
	loop_rate.sleep();
}

void FindCircles::find_blue(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp0-tmp1 > 20 && tmp0-tmp2 > 50 && tmp0 > 80))
			//if (tmp2 > 120 && tmp2 < 255 && tmp1 > 0 && tmp1 < 80 && tmp0 > 0 && tmp0 < 80) //red
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;
	imshow("blue", imgThresholded);
	if (char(waitKey(1)) == 'o')
		destroyWindow("blue");

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 200)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}

	
	if (!contours.empty())
	{
		Rect rb = boundingRect(Mat(contours[0]));
		geometry_msgs::Point midP;
		midP.x = 0.5*(rb.tl().x + rb.br().x);
		midP.y = 0.5*(rb.tl().y + rb.br().y);
		cout << "circle center: " << midP.x << '\t' << midP.y << endl;
	
		midP.x = ardrone_pos.x - midP.x;
		midP.y = ardrone_pos.y - midP.y;
		geometry_msgs::Point dist;
		revmeasurement(midP.x, midP.y, dist.x, dist.y, pitch, roll, height);
		geometry_msgs::Point result;
		result.x = dist.y;
		result.y = dist.x;
		blue_pub.publish(result);
		cout << "blue found: " << result.x << '\t' << result.y << endl;
	}
}

void FindCircles::find_yellow(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp2 > 180 && tmp2 < 240 && tmp1 > 150 && tmp1 < 190 && tmp0 > 0 && tmp0 < 80))
			//if (tmp2 > 120 && tmp2 < 255 && tmp1 > 0 && tmp1 < 80 && tmp0 > 0 && tmp0 < 80) //red
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;
	imshow("yellow", imgThresholded);
	if (char(waitKey(1)) == 'o')
		destroyWindow("yellow");

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 200)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}

	
	if (!contours.empty())
	{
		Rect rb = boundingRect(Mat(contours[0]));
		geometry_msgs::Point midP;
		midP.x = 0.5*(rb.tl().x + rb.br().x);
		midP.y = 0.5*(rb.tl().y + rb.br().y);
		//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
	
		midP.x = ardrone_pos.x - midP.x;
		midP.y = ardrone_pos.y - midP.y;
		geometry_msgs::Point dist;
		revmeasurement(midP.x, midP.y, dist.x, dist.y, pitch, roll, height);
		geometry_msgs::Point result;
		result.x = dist.y;
		result.y = dist.x;
		yellow_pub.publish(result);
		//cout << "yellow : " << result.x << '\t' << result.y << endl;
	}
}

void FindCircles::find_red(Mat &img, Mat imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			//if (tmp2 > 80 && tmp2 < 180 && tmp1 > 0 && tmp1 < 50 && tmp0 > 0 && tmp0 < 70)
			if((tmp2-tmp1) >= 70 && (tmp2-tmp0) >= 60)//day:(tmp2-tmp1) >= 100 && (tmp2-tmp0) >= 100)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;
	imshow("red", imgThresholded);
	if (char(waitKey(1)) == 'o')
		destroyWindow("red");
	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> rb;
	ardrone_trajectory::points real_msg;
	ardrone_trajectory::points image_msg;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 200)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rb.push_back(r);
			rectangle(img, r, Scalar(0, 0, 255), 3);//testing
			++it;
		}
	}

	if (!rb.empty())
	{
		for (int i = 1; i < rb.size(); ++i)
		{
			double d0 = pow(0.5*(rb[0].tl().x+rb[0].br().x-img.cols),2)+pow(0.5*(rb[0].tl().y+rb[0].br().y-img.rows),2);
			double di = pow(0.5*(rb[i].tl().x+rb[i].br().x-img.cols),2)+pow(0.5*(rb[i].tl().y+rb[i].br().y-img.rows),2);
		
			if (d0 > di)
			{
				Rect tmp;
				tmp = rb[0];
				rb[0] = rb[i];
				rb[i] = tmp;
			}
		}
		rectangle(img, rb[0], Scalar(255, 0, 0), 3);
		for (int i = 0; i < rb.size(); ++i)
		{
			geometry_msgs::Point data;
			data.x = 0.5*(rb[i].tl().x + rb[i].br().x);
			data.y = 0.5*(rb[i].tl().y + rb[i].br().y);
			//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
			geometry_msgs::Point img_data;
			img_data.y = data.x - 0.5*img.cols;
			img_data.x = data.y - 0.5*img.rows;
			image_msg.point.push_back(img_data);

			geometry_msgs::Point dist;
			double dx, dy;
			dx = ardrone_pos.x - data.x;
			dy = ardrone_pos.y - data.y;
			//cout << "dx, dy : " << dx << '\t' << dy << endl;
			revmeasurement(dx, dy, dist.x, dist.y, pitch, roll, height);
			geometry_msgs::Point result;
			result.x = dist.y;
			result.y = dist.x;
			//cout << "dist.x, dist.y : " << dist.x << '\t' << dist.y << endl;
			//cout << "result.x, result.y : " << result.x << '\t' << result.y << endl;
			real_msg.point.push_back(result);
		}
		//cout << real_msg.point[0].x << '\t' << real_msg.point[0].y << endl;
		red_image_pub.publish(image_msg);
		red_real_pub.publish(real_msg);	//dian xiangduiyu feiji de weizhi

		/*part of correction*/
		/*double k;
		double b;
		double r;
		geometry_msgs::Point foot;
		if (rb.size() >= 2)
		{
			line_fitting(real_msg, k, b, r, foot);
			//cout << r <<endl;
			//cout << "y = " << k << "x + " << b;
			//cout << "foot : " << foot.x << '\t' << foot.y << endl;
			if (fabs(r) >= 0.5)
				correct_pub.publish(foot);
		}*/
	}
}

void FindCircles::line_fitting(ardrone_trajectory::points reds, double &k, double &b, double &r, geometry_msgs::Point &foot)
{
	double xmean = 0;
	double ymean = 0;
	double x2mean = 0;
	double y2mean = 0;
	double xymean = 0;
	for (int i = 0; i < reds.point.size(); ++i)
	{
		xmean += reds.point[i].x;
		ymean += reds.point[i].y;
		x2mean += reds.point[i].x*reds.point[i].x;
		y2mean += reds.point[i].y*reds.point[i].y;
		xymean += reds.point[i].x*reds.point[i].y;
	}
	xmean /= reds.point.size();
	ymean /= reds.point.size();
	x2mean /= reds.point.size();
	y2mean /= reds.point.size();
	xymean /= reds.point.size();

	double tmp;
	if (tmp = x2mean -xmean*xmean)
	{
		k = (xymean -xmean*ymean)/tmp;
		b = ymean -k*xmean;
	} else {
		k = 1;
		b = 0;
	}
	r = (xymean - xmean*ymean)/sqrt((x2mean - xmean * xmean) * (y2mean - ymean * ymean));
	foot.x = b * sqrt(1/(1+k*k))*sqrt(1/(1+k*k));
	foot.y = k * foot.x + b; 
}

void FindCircles::measurement(double &x, double &y, double u, double v, double pitch, double roll, double h)
{
	double m = u * h / (h/cos(roll) + u*sin(roll));
	double n = v * h / (h/cos(pitch) + u*sin(pitch));
	x = 1000 * m / (1.55 * h + 0.01078);
	y = 1000 * n / (1.55 * h + 0.01078);
}
void FindCircles::revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h)
{
	double m = (1.55 * h + 0.01078) / 1000 * x;
	double n = (1.55 * h + 0.01078) / 1000 * y;
	//ROS_INFO("m =%f, n =%f", m, n);
	u = m * h/cos(roll) / (h-m*sin(roll));
	v = n * h/cos(pitch) / (h-n*sin(pitch));
	//ROS_INFO("u =%f, v =%f", u, v);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "circle_detect");
	FindCircles fc;
	ros::spin();
	return 0;
}