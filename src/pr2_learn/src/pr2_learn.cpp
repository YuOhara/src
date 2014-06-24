#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace ros;
using namespace cv;

#define WIDTH 640
#define HEIGHT 480
#define W_M 350
#define H_M 250

class PR2_learn{
public:
  ros::NodeHandle n;
  ros::Subscriber sub_info;
  image_transport::ImageTransport _it;
  image_transport::Subscriber sub_img;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat *screen;
  typedef struct S_P{
    S_P(){
      fetch = false;
      on_mouse = false;
    }
    S_P(cv::Mat *screen_input){
      fetch = false;
      on_mouse = false;
      screen = screen_input;
    }
    cv::Mat *screen;
    cv::Mat temp_img;
    cv::Rect roi_rect;
    bool fetch;
    bool on_mouse;
    int on_x, on_y;
    std::string name;
  } save_param;
  
  typedef struct M_P{
    M_P(){
      on_mouse = false;
      on_move = 0;
    }
    int temp_x, temp_y;
    int on_x, on_y;
    bool on_mouse;
    int on_move;
    std::vector<save_param> save_data;
  } mouse_param;
  
  mouse_param m_p;
  PR2_learn(): _it(n){
    screen = new cv::Mat(cv::Size(WIDTH+W_M*2, HEIGHT+H_M*2), CV_8UC3, cv::Scalar(0, 0, 0));
    sub_img = _it.subscribe("image_rect_color", 10, &PR2_learn::image_cb, this); //should take camera_info before it ;
    ROS_INFO("START SUBSCRIBING");
    sub_info = n.subscribe("camera_info",10 ,&PR2_learn::info_cb,this);
    m_p.save_data.push_back(save_param(screen));
    ROS_INFO("save_date_size: %d", m_p.save_data.size());
    cv::namedWindow("pr2",1);
    cv::setMouseCallback("pr2", mouse_cb, (void*) &m_p);
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr){
    *screen = cv::Scalar(100, 100, 100);
    try{
      cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Rect roi_rect;
    roi_rect.width=WIDTH;
    roi_rect.height=HEIGHT;
    roi_rect.x += W_M;
    roi_rect.y += H_M;
    cv::Mat roi1(*screen, roi_rect);
    cv_ptr->image.copyTo(roi1);
    show_rect();
    if(m_p.on_move==1){
      cv::rectangle(*screen, cv::Point(WIDTH/2+W_M-90, H_M/2-50), cv::Point(WIDTH/2+W_M+90, H_M/2+10), cv::Scalar(20, 200, 20), -1, CV_AA);
    }
    else if(m_p.on_move==2){
      cv::rectangle(*screen, cv::Point(WIDTH/2+W_M-90, H_M/2-50), cv::Point(WIDTH/2+W_M+90, H_M/2+10), cv::Scalar(20, 20, 200), -1, CV_AA);
    }
    else{
      cv::rectangle(*screen, cv::Point(WIDTH/2+W_M-90, H_M/2-50), cv::Point(WIDTH/2+W_M+90, H_M/2+10), cv::Scalar(200, 20, 20), -1, CV_AA);
    }
    cv::putText(*screen, "Move!", cv::Point(WIDTH/2+W_M - 80, H_M/2), cv::FONT_HERSHEY_PLAIN, 3.5, cv::Scalar(100, 255, 100), 2, CV_AA);
    cv::putText(*screen, "place", cv::Point(W_M/2 - 80, H_M/2), cv::FONT_HERSHEY_PLAIN, 3.5, cv::Scalar(100, 255, 100), 2, CV_AA);
    cv::putText(*screen, "objects", cv::Point(WIDTH+W_M*3/2-80 - 20, H_M/2), cv::FONT_HERSHEY_PLAIN, 3.5, cv::Scalar(100, 255, 100), 2, CV_AA);
    //screen should be copied
    cv::imshow("pr2", *screen);
    cv::waitKey(20); 
  }

  void info_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr){
    ROS_INFO("info_CB");
  }
  static void mouse_cb(int event, int x, int y, int flags, void* param = NULL){
    mouse_param* m_p = (mouse_param*)param;
    int dx = x - m_p->temp_x; int dy = y - m_p->temp_y;
    m_p->temp_x=x; m_p->temp_y=y;
    //(WIDTH/2+W_M-90, H_M/2-50), cv::Point(WIDTH/2+W_M+90, H_M/2+10)
    if(x>WIDTH/2+W_M-90&&x<WIDTH/2+W_M+90&&y>H_M/2-50&&y<H_M/2+10){
      if(m_p->on_move==0){
	m_p->on_move=1;
      }
      if(event==CV_EVENT_LBUTTONDOWN){
	m_p->on_move=2;
      }
      return;
    }
    else{
      m_p->on_move=0;
    }
    for( std::vector<save_param>::iterator m = m_p->save_data.begin(); m!=m_p->save_data.end(); m++){      
      if(m_p->on_mouse) {m = m_p->save_data.end()-1;}
      if(!(x>m->roi_rect.x&&x<(m->roi_rect.x+m->roi_rect.width)&&y>m->roi_rect.y&&y<(m->roi_rect.y+m->roi_rect.height)) &&
	 (m+1) != m_p->save_data.end() && !m->fetch){
	continue;
      }
      if(m->fetch){
	m->roi_rect.x+=dx;
	m->roi_rect.y+=dy;
      }
      switch(event){
      case CV_EVENT_LBUTTONDOWN:
	if(x>m->roi_rect.x&&x<(m->roi_rect.x+m->roi_rect.width)&&y>m->roi_rect.y&&y<(m->roi_rect.y+m->roi_rect.height)){
	  m->fetch = true;
	  ROS_INFO("fetch");
	  ROS_INFO("fetch");
	  if((m+1)!=m_p->save_data.end()) {std::iter_swap(m, m_p->save_data.begin()); ROS_INFO("SWAP");};
	  return;
	}
	else{
	  m->on_x = x, m->on_y=y;
	  m->on_mouse = true;
	  m_p->on_mouse = true;
	  return;
	}
	break;
      case CV_EVENT_LBUTTONUP:
	m_p->on_mouse = false;
	m->on_mouse = false;
	if(m->fetch){
	  m->fetch = false;
	  //if out of screen add_data
	  //if somewhere on dust delete_data
	  //or right click
	  //zero should be specified
	  //	  std::iter_swap(m, m_p->save_data.begin());
	  return;
	}
	else
	  if(m==m_p->save_data.end()-1)
	  {
	    m->roi_rect.width = abs(m_p->temp_x - m->on_x);
	    m->roi_rect.height = abs(m_p->temp_y - m->on_y);
	    m->roi_rect.x = cv::min(m->on_x, m_p->temp_x);
	    m->roi_rect.y = cv::min(m->on_y, m_p->temp_y);
	    cv::Mat roi(*(m->screen), m->roi_rect);
	    roi.copyTo((m->temp_img));
	    //std::iter_swap(m, m_p->save_data.begin());
	    return;
	}
	break;
      case CV_EVENT_RBUTTONDOWN:
	if((m+1)!=m_p->save_data.end()){
	  m_p->save_data.erase(m);
	  return;
	}
	m_p->save_data.push_back(save_param(m->screen));
	return;
	//save do
	break;
      }
    }
  }
  void show_rect(){
    for( std::vector<save_param>::iterator m = m_p.save_data.end()-1; m!=m_p.save_data.begin()-1; m--){      
      if(m->fetch){
	ROS_INFO("fetch!");
      	cv::rectangle(*screen, cv::Point(m->roi_rect.x, m->roi_rect.y), cv::Point(m->roi_rect.x+m->roi_rect.width, m->roi_rect.y+m->roi_rect.height), cv::Scalar(0, 200, 200), 3, 4);
      	cv::Mat roi(*(m->screen), m->roi_rect);
      	((m->temp_img)).copyTo(roi);
      	continue;
      }
      if(m->on_mouse){
      	cv::rectangle(*screen, cv::Point(m_p.temp_x, m_p.temp_y), cv::Point(m->on_x, m->on_y), cv::Scalar(0, 0, 200), 3, 4);
      	continue;
      }
      
      cv::rectangle(*screen, cv::Point(m->roi_rect.x, m->roi_rect.y), cv::Point(m->roi_rect.x+m->roi_rect.width, m->roi_rect.y+m->roi_rect.height), cv::Scalar(0, 200, 200), 3, 4);
      cv::Mat roi(*(m->screen), m->roi_rect);
      ((m->temp_img)).copyTo(roi);
      continue;
    }
  }
};
int main(int argc, char** argv){
  ros::init(argc, argv, "particla_detect");
  ROS_INFO("START TRACKING"); 
  PR2_learn PC;
  ros::spin();

  return 0;
}
