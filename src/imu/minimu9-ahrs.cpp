#include "mini_project/vector.h"
#include "mini_project/version.h"
#include "mini_project/prog_options.h"
#include "mini_project/minimu9.h"
#include "mini_project/exceptions.h"
#include "mini_project/pacer.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <system_error>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"


typedef void rotation_output_function(quaternion & rotation);
void fill_unknowns(sensor_msgs::Imu&);
void fill_header(sensor_msgs::Imu&);
void fill_mag(sensor_msgs::MagneticField&);

int main(int argc, char **argv) {

	/* Begin Shit I dont Understand */
	char *fake_options[3] = {
		(char*) "minimu9-ahrs",
		(char*) "--mode",
		(char*) "raw"
		};

	prog_options options = get_prog_options(3, fake_options);
	if(options.show_help)
	{
		print_command_line_options_desc();
		std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
		return 0;
	}

	if (options.show_version)
	{
		std::cout << "fuck you " << VERSION << std::endl;
		return 0;
	}

sensor_set set;
	set.mag = set.acc = set.gyro = true;

	minimu9::comm_config config = minimu9::auto_detect(options.i2c_bus_name);

	sensor_set missing = set - minimu9::config_sensor_set(config);
	if (missing)
	{
		if (missing.mag)
		{
			std::cerr << "Error: No magnetometer found." << std::endl;
		}
		if (missing.acc)
		{
			std::cerr << "Error: No accelerometer found." << std::endl;
		}
		if (missing.gyro)
		{
			std::cerr << "Error: No gyro found." << std::endl;
		}
			std::cerr << "Error: Needed sensors are missing." << std::endl;
		return 1;
	}
	config = minimu9::disable_redundant_sensors(config, set);

	minimu9::handle imu;
	imu.open(config);

	rotation_output_function * output;
	/* end shit I don't understand */

	ROS_INFO("%s", "Fetching raw imu vals\n");
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
	//ros::Rate loop_rate(200);
	imu.enable();

	double gyro_scale = 0.004375 * 3.14159265 / 180.;
	double acro_scale = 0.000061 * 9.81;
	double offset_acro[3] = {-306.785400, 1823.514526, -413.978755951};
	double offset_gyro[3] = {-180.016068, 519.858459, 411.838379};
	sensor_msgs::Imu imu_msg;

	/* comment this toggle claibration */
//	int count = 0;
//	/* number of avg values */
//	int iter = 20000;
//	/* sleep just in case */
//	float sleep_duration = 0;
//	ros::Duration(sleep_duration)::Sleep();
//	float a_acumlat[3] = {0, 0, 0};
//	float a_avg[3] = {0, 0, 0};
//	float g_acumlat[3] = {0, 0, 0};
//	float g_avg[3] = {0, 0, 0};
//
//	for (int i = 0; i < iter; ++i) {
//		count++;
//		imu.read_raw();
//		for (int i = 0; i < 3; ++i) {
//			a_acumlat[i] += imu.a[i];
//			g_acumlat[i] += imu.g[i];
//			a_avg[i] = a_acumlat[i] / count;
//			g_avg[i] = g_acumlat[i] / count;
//		}
//	}
//	offset_acro[0] = -a_avg[0];
//	offset_acro[1] = -a_avg[1];
//	offset_acro[2] = -1 / 0.000061 - a_avg[2];
//
//	offset_gyro[0] = -g_avg[0];
//	offset_gyro[1] = -g_avg[1];
//	offset_gyro[2] = -g_avg[2];
//
//	ROS_INFO("a avg: %5f %5f %5f g avg: %5f %5f %5f\n",
//				a_avg[0], a_avg[1], a_avg[2],
//				g_avg[0], g_avg[1], g_avg[2]
//		);
	/* end toggle calibration */
	while(ros::ok()) {
		imu.read_raw();

		imu_msg.angular_velocity.x = (offset_gyro[0] + imu.g[0]) * gyro_scale;
		imu_msg.angular_velocity.y = (offset_gyro[1] + imu.g[1]) * gyro_scale;
		imu_msg.angular_velocity.z = (offset_gyro[2] + imu.g[2]) * gyro_scale;

		imu_msg.linear_acceleration.x = (offset_acro[0] + imu.a[0]) * acro_scale;
		imu_msg.linear_acceleration.y = (offset_acro[1] + imu.a[1]) * acro_scale;
		imu_msg.linear_acceleration.z = (offset_acro[2] + imu.a[2]) * acro_scale;

//		printf("a: %5d %5d %5d \t g: %5d %5d %5d\n %5f %5f %5f \t g: %5f %5f %5f\n",
//				imu.a[0], imu.a[1], imu.a[2],
//				imu.g[0], imu.g[1], imu.g[2],
//				imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
//				imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

		fill_unknowns(imu_msg);
		fill_header(imu_msg);
		imu_pub.publish(imu_msg);
		ros::spinOnce();
		//loop_rate.sleep();
		
//		count++;
//		for (int i = 0; i < 3; ++i) {
//			a_acumlat[i] += imu.a[i];
//			g_acumlat[i] += imu.g[i];
//			a_avg[i] = a_acumlat[i] / count;
//			g_avg[i] = g_acumlat[i] / count;
//		}
//		ROS_INFO("a avg: %5f %5f %5f g avg: %5f %5f %5f\n",
//				a_avg[0], a_avg[1], a_avg[2],
//				g_avg[0], g_avg[1], g_avg[2]
//			);
	}
	return 0;
}

void fill_unknowns(sensor_msgs::Imu &msg)
{
	msg.orientation.x = 0;
	msg.orientation.y = 0;
	msg.orientation.z = 0;
	msg.orientation.w = 0;

	for (int i = 0; i < 9; ++i) {
		msg.orientation_covariance[i] = 0;
		msg.angular_velocity_covariance[i] = 0;
		msg.linear_acceleration_covariance[i] = 0;
	}
}

void fill_header(sensor_msgs::Imu &msg)
{
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "imu_raw_frame";
}


void fill_mag(sensor_msgs::MagneticField& msg)
{
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "imu_mag_frame";

	for (int i = 0; i < 9; ++i)
		msg.magnetic_field_covariance[i] = 0;

	msg.magnetic_field.x = std::numeric_limits<double>::quiet_NaN();
	msg.magnetic_field.y = std::numeric_limits<double>::quiet_NaN();
	msg.magnetic_field.z = std::numeric_limits<double>::quiet_NaN();
}
