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
sensor_msgs::PointCloudConstPtr cur_img;

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
    cur_img=img;
    return true;
    
  }
  return false;
}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    if(!getImufromFile(imuFile)) return measurements;
    if(!getImagefromFile(imageFile)) return measurements;
    cout<<setprecision(20)<<imu_buf.back().header.stamp.toSec()<<" "<<cur_img.header.stamp.toSec()<<endl;

        while(true)
        {
             if (!(imu_buf.back().header.stamp.toSec() > cur_img.header.stamp.toSec() + 0.001))
            {
            sum_of_wait++;
            if(!getImufromFile(imuFile)) return measurements;
            continue;
           }
        if (!(imu_buf.front().header.stamp.toSec() < cur_img.header.stamp.toSec() + 0.001))
           {
            if(!getImagefromFile(imageFile)) return measurements;
            continue;
            }

          std::vector<sensor_msgs::ImuConstPtr> IMUs;
          while (imu_buf.front().header.stamp.toSec() < cur_img.header.stamp.toSec() + 0.001)
           {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
            }
          IMUs.emplace_back(imu_buf.front());
          measurements.emplace_back(IMUs, cur_img);
          return measurements;
        }

 
}


int main()
{
  while(true)
  {
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
    measurements=getMeasurements();
    if(measurements.size()==0) break;
    for(int i=0;i<measurements.size();i++)
    {
      cout<<i<<":"<<endl;
      cout<<"imu"<<endl;
        for(int j=0;j<measurements[i].first.size();j++)
        {
          cout<<setprecision(20)<<measurements[i].first[j].header.stamp.toSec()<<endl;
        }
        cout<<"cam"<<endl;
        cout<<measurements[i].second.header.stamp.toSec()<<endl;
    }
    cin.get();
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