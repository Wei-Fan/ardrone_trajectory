#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
#include "Eigen/Dense"
#include "Eigen/Geometry" 

using namespace std;
using namespace Eigen;

class YAW_EST
{
public: 
	YAW_EST();
private:
	ros::NodeHandle n;
	ros::Subscriber navdata_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber mag_sub;
	ros::Publisher yaw_pub;

	double roll;
	double pitch;
	double yaw;

	double yaw_mag;
	double buf[20];
	int buf_ptr;

	Vector3f mag_body;
	Vector3f mag_earth;
	Quaternionf Q;
	Matrix3f R;

	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void imuCallback(const sensor_msgs::Imu &msg);
	void magCallback(const geometry_msgs::Vector3Stamped &msg);
};

YAW_EST::YAW_EST()
{
	navdata_sub = n.subscribe("/ardrone/navdata", 1, &YAW_EST::navdataCallback, this);
	imu_sub = n.subscribe("ardrone/imu", 1, &YAW_EST::imuCallback, this);
	mag_sub = n.subscribe("ardrone/mag", 1, &YAW_EST::magCallback, this);
	yaw_pub = n.advertise<std_msgs::Float32>("/ardrone/yaw", 1);

	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;
	memset(buf, 0, sizeof(buf));
	buf_ptr = 0;
}

void YAW_EST::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	//ROS_INFO("rotZ: %f", msg.rotZ);
}

void YAW_EST::magCallback(const geometry_msgs::Vector3Stamped &msg)
{
	mag_body(0) = msg.vector.x;
	mag_body(1) = msg.vector.y;
	mag_body(2) = msg.vector.z;

	yaw_mag = atan2f(-mag_body(0),-mag_body(1));

	buf[buf_ptr] = yaw_mag;
	double yaw_mean = 0;
	for(int i=0; i<20; i++)
	{
		yaw_mean += buf[i];
	}
	yaw_mean /= 20;
	buf_ptr++;
	if(buf_ptr > 19)
	{
		buf_ptr = 0;
	}
	std_msgs::Float32 ret;
	ret.data = yaw_mean;
	yaw_pub.publish(ret);
	//ROS_INFO("yaw: %f", yaw_mean*57.3);
}

void YAW_EST::imuCallback(const sensor_msgs::Imu &msg)
{
	double q0,q1,q2,q3;
	q0 = msg.orientation.w;
	q1 = msg.orientation.x;
	q2 = msg.orientation.y;
	q3 = msg.orientation.z;
	Q = Quaternionf(q0,q1,q2,q3);
	R = Q.toRotationMatrix(); 

	pitch = atan2(2*(q2*q3-q0*q1), q0*q0-q1*q1-q2*q2+q3*q3);
	roll = asin(-2*(q0*q2+q1*q3));
	yaw = atan2(2*(q1*q2-q0*q3), q0*q0+q1*q1-q2*q2-q3*q3);
	//ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll*57.3, pitch*57.3, yaw*57.3);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "yaw_est");
	YAW_EST yaw_est;
	ros::spin();
}