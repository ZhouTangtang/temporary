#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <string>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "sensor_msgs.h"

using namespace std;
std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

vector<string> split(const string &s, const string &seperator);

void readImufromFile(const string &filename)
{
    ifstream ifs(filename);
    string line;
    int i=0;
    while(ifs.good())
    {
        getline(ifs,line);
        if(line[0]=='#') continue;
        vector<string> buffer=split(line,",");
        if(buffer.size()<7) continue;
        sensor_msgs::Imu imu;
        long long time=stoll(buffer[0].c_str());
        imu.header.stamp.secs=(int)(time/1e9);
        imu.header.stamp.nsecs=time%1000000000;
        imu.header.seq=i++;
        imu.angular_velocity.x=stod(buffer[1].c_str());
        imu.angular_velocity.y=stod(buffer[2].c_str());
        imu.angular_velocity.z=stod(buffer[3].c_str());
        imu.linear_acceleration.x=stod(buffer[4].c_str());
        imu.linear_acceleration.y=stod(buffer[5].c_str());
        imu.linear_acceleration.z=stod(buffer[6].c_str());
        //imu_buf.push(imu);
        sensor_msgs::ImuConstPtr imuptr=std::make_shared<sensor_msgs::Imu const>(imu);
        imu_buf.push(imuptr);
    }

}


vector<string> split(const string &s, const string &seperator){
  vector<string> result;
  typedef string::size_type string_size;
  string_size i = 0;
  
  while(i != s.size()){
    int flag = 0;
    while(i != s.size() && flag == 0){
      flag = 1;
      for(string_size x = 0; x < seperator.size(); ++x)
    if(s[i] == seperator[x]){
      ++i;
      flag = 0;
      break;
    }
    }
    
    flag = 0;
    string_size j = i;
    while(j != s.size() && flag == 0){
      for(string_size x = 0; x < seperator.size(); ++x)
    if(s[j] == seperator[x]){
      flag = 1;
      break;
    }
      if(flag == 0) 
    ++j;
    }
    if(i != j){
      result.push_back(s.substr(i, j-i));
      i = j;
    }
  }
  return result;
}