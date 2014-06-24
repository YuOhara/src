#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "posedetection_msgs/ObjectDetection.h" //later, it may be changed
#include <iostream>
#include <math.h>

#define X_I 1
#define Y_I -3
#define X_SIZE 10
#define Y_SIZE 7

typedef struct St{
  //state
  bool free; //not
  bool detected;
  bool memory; //later, memory and expand will be converted //not
  bool on_camera;
  float size;
  //  float time;
  //  float memory;
  St(){
    free = 1;
    detected = 0;
    memory = 0;
    on_camera = 0;
    //    size=0;
    //    memory = 0.001; // memory will be 正規化
  }
  
} State;

class Marker{
public:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber sift_sub;
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker>::iterator marker_itr;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  float camera_vec[3];
  float temp_x, temp_y;
  State state[X_SIZE*Y_SIZE] ;
  int counter;
  Marker(){
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array_expand3", 10);
    sift_sub = n.subscribe("openni/rgb/ObjectDetection", 10, &Marker::sift_cb, this); 
    init();
  }
  void init(){
    for (int i=0 ; i<X_SIZE; i++){
      for (int j=0 ; j<Y_SIZE; j++){
	//	state[i*Y_SIZE+j]=0;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "eng2/7f/73B2";
	char s[16];
	sprintf(s, "mymame %d %d", i, j);
	marker.ns = s;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = i+X_I;
	marker.pose.position.y = j+Y_I;
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;//1;
	marker.scale.y = 0.1;//1;
	marker.scale.z = 0.1;//1;
	marker.color.a = 0.1;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	marker_array.markers.push_back(marker);
      }
    }
    state[35].memory=true;
  }
  void pub(){
    state_calc();
    marker_pub.publish(marker_array);
  }
  void state_calc(){
    try{                                                                                              
      listener.lookupTransform("/eng2/7f/73B2", "/openni_rgb_optical_frame"/*"/head_mount_kinect_rgb_link"*/, ros::Time(0), transform);               
      temp_x=transform.getOrigin().x();
      temp_y=transform.getOrigin().y();
      float w = transform.getRotation().w();
      float x = transform.getRotation().x();
      float y = transform.getRotation().y();
      float z = transform.getRotation().z();
      // camera_vec[0]=w*w+x*x-y*y-z*z; //if x
      // camera_vec[1]=2*w*z+2*x*y;
      // camera_vec[2]=2*x*z-2*w*y;
      camera_vec[0]=2*w*y+2*x*z; //if z
      camera_vec[1]=-2*x*w+2*y*z;
      camera_vec[2]=w*w+z*z-x*x-y*y;

      ROS_INFO("temp_x:%f temp_y:%f, innner=%f(=1)", temp_x, temp_y, camera_vec[0]*camera_vec[0]+camera_vec[1]*camera_vec[1]+camera_vec[2]*camera_vec[2]);
      ROS_INFO("camera :: x:%f y:%f z:%f", camera_vec[0], camera_vec[1], camera_vec[2]);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      temp_x = -100;
    }
    counter = 0;
    for(marker_itr=marker_array.markers.begin(); marker_itr!=marker_array.markers.end(); marker_itr++)
      {
	marker_itr-> header.stamp = ros::Time();
	marker_itr->color.r=1.0; marker_itr->color.g=1.0; marker_itr->color.b=0.0;

	expand();
	wither_by_camera();
	expand_by_sift();
	memory();
	size_calc();
	counter++;
      }
  }
  void expand(){
    state[counter].size+=0.0003;

    if(state[counter].size>.5){
      state[counter].size = 0.5;
    }
  }
  float dist(float x1, float x2, float y1, float y2){
    return (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
  }
  void wither_by_camera(){
    float vx = marker_itr->pose.position.x - temp_x;
    float vy = marker_itr->pose.position.y - temp_y;
    float distance = dist(marker_itr->pose.position.x, temp_x, marker_itr->pose.position.y, temp_y);
    if(distance < 13){
      if((vx*camera_vec[0]+vy*camera_vec[1])/distance <0.3) return;
      state[counter].size-=0.001;
	if(state[counter].size<=0){
	    state[counter].size = 0.01;
	    state[counter].on_camera = true;
        }
      marker_itr->color.r=1.0; marker_itr->color.g=0.0; marker_itr->color.b=0.0;
    }
    else{
      state[counter].on_camera = false;
    }
  }
  void expand_by_sift(){
    if(state[counter].detected){
      state[counter].size -= 0.0001;
      marker_itr->color.r=.0; marker_itr->color.g=1.0; marker_itr->color.b=1.0;
      if(state[counter].size < 0.5){
	state[counter].detected = false;
	state[counter].memory = true;
      }
    }    
  }
  void memory(){
    if(state[counter].memory){
      marker_itr->color.r=0.0; marker_itr->color.g=0.0; marker_itr->color.b=1.0;
      state[counter].size = 0.5;
    }
  }
  void sift_cb(const posedetection_msgs::ObjectDetection& msg){   
    ROS_INFO("detected");
    ROS_INFO("detected");
    tf::Quaternion q(msg.objects[0].pose.position.x, msg.objects[0].pose.position.y, msg.objects[0].pose.position.z, 0);
    q = transform.getRotation().inverse() *q * transform.getRotation();
    q.x();
    ROS_INFO("point %f %f", (float)(q.x()+temp_x) ,  (float)( q.y()+temp_y));
    int point = (((int)(q.x()+temp_x+0.5))-X_I) * Y_SIZE + (int)( q.y()+temp_y +0.5)-Y_I; 
    float point_f = (((float)(q.x()+temp_x+0.5))-X_I) * Y_SIZE + (float)( q.y()+temp_y +0.5)-Y_I; 
    state[point].detected = true;
    state[counter].size = 1.0;
    ROS_INFO("point: %d", point);
    return;
  }
  void size_calc(){
    marker_itr->scale.x=state[counter].size; marker_itr->scale.y=state[counter].size ; marker_itr->scale.z = state[counter].size;
  }
  ~Marker(){
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "marker_display");
  Marker *m = new Marker();
  ros::Rate loop_rate(10);
  while(ros::ok()){
    m->pub();
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("version 1.01");
    ROS_INFO("published markers");
  }
}
