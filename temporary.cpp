#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <string>
#include <condition_variable>
#include <fstream>
#include <iomanip>
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

ifstream imuFile("/home/zhouyuxuan/data/MH_03_medium/mav0/imu0/data.csv");
ifstream imageFile("/home/zhouyuxuan/data/MH_03_medium/mav0/cam0/data.csv");
string imagePath="/home/zhouyuxuan/data/MH_03_medium/mav0/cam0/data";

vector<string> split(const string &s, const string &seperator);

//
bool getImufromFile(std::ifstream &ifs)
{
   // ifstream ifs(filename);
    string line;
    while(ifs.good())
    {
        getline(ifs,line);
        if(line[0]=='#') continue;
        vector<string> buffer=split(line,",");
        if(buffer.size()<7) continue;
        sensor_msgs::ImuConstPtr imu;
        long long time=stoll(buffer[0].c_str());
        imu.header.stamp.secs=(int)(time/1e9);
        imu.header.stamp.nsecs=time%1000000000;
        imu.header.frame_id="world";
        imu.angular_velocity.x=stod(buffer[1].c_str());
        imu.angular_velocity.y=stod(buffer[2].c_str());
        imu.angular_velocity.z=stod(buffer[3].c_str());
        imu.linear_acceleration.x=stod(buffer[4].c_str());
        imu.linear_acceleration.y=stod(buffer[5].c_str());
        imu.linear_acceleration.z=stod(buffer[6].c_str());
        imu_buf.push(imu);
        return true;
    }
    return false;
}
bool getImagefromFile(std::ifstream &ifs)
{
  string line;
  while(ifs.good())
  {
    getline(ifs,line);
    if(line[0]=='#') continue;
    vector<string> buffer=split(line,",");
    if(buffer.size()<2) continue;
    sensor_msgs::PointCloudConstPtr img;
    long long time=stoll(buffer[0].c_str());
    img.header.stamp.secs=(int)(time/1e9);
    img.header.stamp.nsecs=time%1000000000;
    img.header.frame_id="world";
    img.filename=imagePath+"/"+buffer[1];
    //
    feature_buf.push(img);
    return true;
    
  }
  return false;
}




std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back().header.stamp.toSec() > feature_buf.front().header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            getImufromFile(imuFile);
            continue;
        }
        if (!(imu_buf.front().header.stamp.toSec() < feature_buf.front().header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            getImagefromFile(imageFile);
            continue;
        }


        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front().header.stamp.toSec() < img_msg.header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            //ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}


int main()
{
  while(getImagefromFile(imageFile))
  {
    cout<<setprecision(20)<<feature_buf.back().header.stamp.toSec()<<" "<<feature_buf.back().filename<<endl;
  }
  cin.get();
  return 0;
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