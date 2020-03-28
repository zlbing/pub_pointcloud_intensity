//
// Created by zzz on 19-2-21.
//


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud.h>

std::vector<Eigen::Vector4f> g_origin_points;
std::vector<Eigen::Vector4f> origin_part_points;
float g_min_x=1e6, g_max_x=-1e6, g_min_y=1e6, g_max_y=-1e6;

ros::Publisher pub_all_points;

int g_pub_count=0;

void EigenVectorToMsg(const std::vector<Eigen::Vector4f>& points,
                      sensor_msgs::PointCloud& points_msg){
  for(int i=0; i<points.size(); i++){
    geometry_msgs::Point32 p;
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = points[i][2];
    float intensity = boost::lexical_cast<float>(points[i][3]);
    points_msg.points.push_back(p);
    points_msg.channels[0].values.push_back(intensity);
  }
}

void PublishPointCloud(const ::ros::WallTimerEvent& unused_timer_event){
  if(pub_all_points.getNumSubscribers() > 0 && g_pub_count <1){

    sensor_msgs::PointCloud all_point_cloud;
    all_point_cloud.header.stamp = ::ros::Time::now();
    all_point_cloud.header.frame_id = "world";
    all_point_cloud.channels.resize(1);
    all_point_cloud.channels[0].name = "intensity";

    EigenVectorToMsg(origin_part_points, all_point_cloud);
    pub_all_points.publish(all_point_cloud);
      g_pub_count++;
    std::cout<<"[PublishPointCloud] ground points publish successful"<<std::endl;
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "intensity_test");
  ros::NodeHandle nh("~");

  std::string filename;
  nh.param(std::string("file_path"),filename,std::string("/home/zzz/groundTexture.pcd"));
  ROS_INFO_STREAM("filename="<<filename<<" in");

  std::ifstream pointcloud_file;
  pointcloud_file.open(filename.c_str(), std::ios::in);
  if(pointcloud_file.is_open()){
    std::string str;
    while (std::getline(pointcloud_file, str, '\n')) {
      std::vector<std::string> str_vec;
      boost::split(str_vec, str, boost::is_any_of(" "));
      if(str_vec[0] == "DATA"){
        break;
      }
    }

    while (std::getline(pointcloud_file, str, '\n')) {
      std::vector<std::string> str_vec;
      boost::split(str_vec, str, boost::is_any_of(" "));
      Eigen::Vector4f p;
      p[0] = boost::lexical_cast<float>(str_vec[0]);
      p[1] = boost::lexical_cast<float>(str_vec[1]);
      p[2] = boost::lexical_cast<float>(str_vec[2]);
      p[3] = boost::lexical_cast<float>(str_vec[3]);
      g_origin_points.emplace_back(p);
      g_min_x = (g_min_x < p[0])?g_min_x:p[0];
      g_max_x = (g_max_x > p[0])?g_max_x:p[0];
      g_min_y = (g_min_y < p[1])?g_min_y:p[1];
      g_max_y = (g_max_y > p[1])?g_max_y:p[1];
    }
    pointcloud_file.close();
  }else{
    ROS_ERROR_STREAM(" lane line file can't open "<<filename);
  }
  std::cout<<"g_min_x="<<g_min_x<<" g_max_x="<<g_max_x
           <<"g_min_y="<<g_min_y<<" g_max_y="<<g_max_y<<std::endl;
  std::cout<<"ground points read successful"<<std::endl;
  pub_all_points = nh.advertise<sensor_msgs::PointCloud>("all_origin_points",10);
  
  int x_seg = 1;
  int y_seg = 1;
  //divide into x_seg*y_seg slices.
  int seg_index  =  0;

  float x_seg_num = (g_max_x - g_min_x)/x_seg;
  float y_seg_num = (g_max_y - g_min_y)/y_seg;
  float x_min = g_min_x + x_seg_num* (seg_index/y_seg);
  float x_max = g_min_x + x_seg_num* ((seg_index/y_seg) + 1);
  float y_min = g_min_y + y_seg_num* (seg_index%y_seg);
  float y_max = g_min_y + y_seg_num* ((seg_index%y_seg)+1);
  for(auto& point : g_origin_points){
    if(point[0]>=x_min && point[0]<=x_max && 
      point[1]>=y_min && point[1]<=y_max){
      origin_part_points.emplace_back(point);
    }
  }
  ::ros::WallTimer wall_timers = nh.createWallTimer(
          ::ros::WallDuration(0.5), PublishPointCloud);

  ros::spin();
}