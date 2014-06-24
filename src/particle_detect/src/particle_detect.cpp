#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace ros;

#define SIZE 500
#define SIZE_X 480
#define SIZE_Y 640
#define varience 13

double Uniform(void){
  return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

double rand_normal(double mu, double sigma){
  double z=sqrt( -2.0*log(Uniform()) ) * sin( 2.0*M_PI*Uniform() );
  return mu + sigma*z;
}


class Point_Class{
public:
  int *x;
  int *y;
  int *r;
  int *x_temp;
  int *y_temp;
  int *r_temp;
  ros::NodeHandle n;
  ros::Publisher point_pub;
  ros::Subscriber sub_info;
  image_transport::ImageTransport _it;
  image_transport::Subscriber sub_img;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat hsv_img;
  int max_index;
  double weight[SIZE];
  float template_vec[16][16];
  Point_Class(): _it(n){
    init_template();
    x = (int *)malloc(sizeof(int)*SIZE);
    y = (int *)malloc(sizeof(int)*SIZE);
    r = (int *)malloc(sizeof(int)*SIZE);
    x_temp = (int *)malloc(sizeof(int)*SIZE);
    y_temp = (int *)malloc(sizeof(int)*SIZE);
    r_temp = (int *)malloc(sizeof(int)*SIZE);
    for (int i=0; i<SIZE; i++){
      x[i]=(int)(Uniform()*SIZE_X);
      y[i]=(int)(Uniform()*SIZE_Y);
      weight[i]=1;
      if(SIZE_X>SIZE_Y){
	r[i]=(int)(Uniform()*SIZE_Y)/3;
      }
      else{
	r[i]=(int)(Uniform()*SIZE_X)/3;
      }
    }
    sub_img = _it.subscribe("image_rect_color", 10, &Point_Class::image_cb, this); //should take camera_info before it ;
    ROS_INFO("START SUBSCRIBING");
    sub_info = n.subscribe("camera_info",10 ,&Point_Class::info_cb,this);
  }
  void init_template(){
    for (int i=0; i<16; i++) for (int j=0; j<16; j++){template_vec[i][j]=0;}
    cv::Mat src_img = cv::imread("/home/ohara/photos/test3.jpg");
    cv::Mat hsv_img_temp;
    cv::cvtColor(src_img, hsv_img_temp, CV_BGR2HSV);
    int size_all = hsv_img_temp.rows*hsv_img_temp.cols;
    ROS_INFO("size_all %d", size_all);
    for(int i=0; i<hsv_img_temp.rows; i++){
      for(int j=0; j<hsv_img_temp.cols; j++){
	uchar h=hsv_img_temp.data[i*hsv_img_temp.step+j*3+0];
	uchar s=hsv_img_temp.data[i*hsv_img_temp.step+j*3+1];
	int count_h=0;
	int count_s=0;
	while(h>15)
	  {
	    h-=16;count_h++;
	  }
	while(s>15)
	  {
	    s-=16;count_s++;
	  }
	template_vec[count_h][count_s] += 1.0/size_all;
      } 
    }
    cv::imshow("template", hsv_img_temp);
    cv::waitKey(20);
  }
  ~Point_Class(){
    free(x);
    free(y);
    free(r);
    free(x_temp);
    free(y_temp);
    free(r_temp);
  }
  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr){
    ROS_INFO("CALL_BACK");
    try{
      cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return ;
    }
    cv::cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);
    ROS_INFO("step=%d, cols=%d", (int)hsv_img.step, hsv_img.cols);
    renew();
    for(int i=0; i<SIZE;i++){
      cv::Point2f pt(y[i], x[i]);
      cv::circle(hsv_img, pt, 1, cv::Scalar(255, 255, 255), 1);
    }
    ROS_INFO("r[i] %d" , r[0]);
    cv::Point2f pt(y[max_index], x[max_index]);
    cv::circle(hsv_img, pt, r[max_index], cv::Scalar(255, 255, 255), 2);
    cv::imshow("result", hsv_img);

    cv::waitKey(20);
  }
  void renew(){
    resample();
    predict();
    weight_calc();
  }
  void info_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr){
    ROS_INFO("INFO_CALLBACK");
    return ;
  }
  void resample(){
    double sum_weights[SIZE];
    sum_weights[0]=weight[0];
    for (int i=1; i<SIZE; i++){
      sum_weights[i]=sum_weights[i-1]+weight[i];
    }
    int *temp;
    temp=x_temp;
    x_temp=x;
    x=temp;
    temp=y_temp;
    y_temp=y;
    y=temp;
    temp=r_temp;
    r_temp=r;
    r=temp;
    for(int i=0; i<SIZE; i++){
      const double weight_temp=Uniform()*sum_weights[SIZE-1];
      int n = 0;
      while(sum_weights[++n] < weight_temp);
      x[i] = x_temp[n];
      y[i] = y_temp[n];
      r[i] = r_temp[n];
    }
  }
  void predict(){
    for(int i=0; i<SIZE; i++){
      int dx=(int)rand_normal(0, varience);
      int dy=(int)rand_normal(0, varience);
      int dr=(int)rand_normal(0, varience);
      if(dx+x[i]<SIZE_X && dx+x[i]>=0){
	x[i]+=dx;
      }
      //else{
      //x[i]+=-dy;
	// }
      if(dy+y[i]<SIZE_Y && dy+y[i]>=0){
	y[i]+=dy;
      }
      // else{
      // 	y[i]+=-dy;
      //}
      if(dr+r[i]<SIZE_X/3-1 && dr+r[i]>=5){
      	r[i]+=dr;
      }
    }
  }
  void measure(){
    return;
  }
  double likelihood(int i){    
    //    return 1;
    int size_all=0;
    int vote_i[16][16];
    for (int j=0; j<16; j++) 
      for (int k=0; k<16; k++){vote_i[j][k]=0;}
    for (int j=-r[i]+x[i]; j<r[i]+x[i]+1; j++){
      for (int k=-r[i]+y[i]; k<r[i]+y[i]+1; k++){
	if ( (j<0) || (j >= hsv_img.rows) || (k<0) || (k >=hsv_img.cols))
	  continue;
	//uchar h = hsv_img.at(j, k); uchar s = h;
	uchar h=hsv_img.data[j*hsv_img.step+k*3+0];
	uchar s=hsv_img.data[j*hsv_img.step+k*3+1];
	int count_h=0;
	int count_s=0;
	while(h>15)
	  {
	    h-=16;count_h++;
	  }
	while(s>15)
	  {
	    s-=16;count_s++;
	  }
	vote_i[count_h][count_s] += 1;
	size_all++;
      }
    }
    if(size_all==0) return 0;
    double result = 0;
    for(int j=0; j<16; j++) 
      for (int k=0; k<16; k++){
	//if(i==0)
	  //std::cout << " " << vote_i[j][k];
	result+=sqrt(template_vec[j][k]*vote_i[j][k]/(double)(size_all));
      }
    if(i==2) ROS_INFO("vec %f %d", template_vec[2][3], vote_i[2][3]);
    if(i==2) ROS_INFO("result %f size_all %d", result, size_all);
    return result;
  }
  void weight_calc(){
    double sum_weight = 0;
    double max_weight = 0;
    int i;
#pragma omp parallel for reduction(+:sum_weight)
    for (i=0; i<SIZE; i++){
      weight[i]=likelihood(i);
      if(weight[i] > max_weight){
	max_weight=weight[i];
	max_index=i;
      }
      sum_weight += weight[i];
    }
    for (int i=0; i<SIZE; i++){
      weight[i]/=(sum_weight/SIZE) ;
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "particla_detect");
  ROS_INFO("START TRACKING"); 
  Point_Class PC;
  ros::spin();

  return 0;
}
