#pragma once

namespace sensor_msgs
{

	// Header
	struct Stamp
	{

		int secs;
	    int nsecs;
		double toSec() const
		{
			double time_in_nsec = secs + (nsecs * 1.0) / 1e9;
			return time_in_nsec;
		}
	};

	struct Header
	{
		int seq;
		Stamp stamp;
		string frame_id;
	};

	// Imu_Msg
	struct Linear_acceleration
	{
		double x;
		double y;
		double z;
	};
	struct Angular_velocity
	{
		double x;
		double y;
		double z;
	};

	struct Quaternion
	{
		double x;
		double y;
		double z;
		double w;
	};

	struct ImuConstPtr
	{
		Header header;
		Quaternion orientation;
		Linear_acceleration linear_acceleration;
		Angular_velocity angular_velocity;
		double orientation_covariance[9];
		double angular_velocity_covariance[9];
		double linear_acceleration_covariance[9];
	};


	// Img_msg
	struct PointCloudConstPtr
	{
		Header header;
		cv::Point3f points;
		vector <double> channels;
	};

};