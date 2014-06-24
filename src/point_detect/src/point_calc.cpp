#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#define SIZE 10000
#define varience 0.5
#define range 10


double Uniform(void){
  return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

double rand_normal(double mu, double sigma){
  double z=sqrt( -2.0*log(Uniform()) ) * sin( 2.0*M_PI*Uniform() );
  return mu + sigma*z;
}

class Point_Class{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp;
  sensor_msgs::PointCloud2 p_msg;
  ros::NodeHandle n;
  ros::Publisher point_pub;// = n.advertise<sensor_msgs::PointCloud2>("prob_point", 10);
  ros::Rate loop_rate; 
  double weight[SIZE];
  Point_Class(): cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_temp(new pcl::PointCloud<pcl::PointXYZ>), loop_rate(10){
    point_pub = n.advertise<sensor_msgs::PointCloud2>("prob_point", 10);
    srand((unsigned int)time(NULL));
    cloud->width=SIZE; cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    cloud_temp->width=SIZE; cloud_temp->height=1;
    cloud_temp->points.resize(cloud_temp->width*cloud_temp->height);
    for(int i=0; i<SIZE; i++){
      weight[i]=1;
      cloud->points[i].x = 2*Uniform() - 1;
      cloud->points[i].y = 2*Uniform() - 1;
      cloud->points[i].z = 2*Uniform() - 1; 
    }
  }
  void resample(){
    double weights[SIZE];
    weights[0]=weight[0];
    for(int i=1; i<SIZE; i++){
      weights[i]=weights[i-1]+weight[i];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2;
    cloud_temp2=cloud_temp;
    cloud_temp=cloud;
    cloud=cloud_temp2;
    ROS_INFO("weights: %f", weights[SIZE-1]);
    for(int i=0; i<SIZE; i++){
      const double weight_temp=Uniform()*weights[SIZE-1];
      int n = 0;
      while(weights[++n] < weight_temp);
      if(i==0) {
	ROS_INFO("n-1 %f t %f n %f", weights[n-1] ,weight_temp, weights[n]);
	ROS_INFO("x:%f, y:%f, z:%f", cloud_temp->points[i].x, cloud_temp->points[i].y, cloud_temp->points[i].z);
      }
      cloud->points[i].x = cloud_temp->points[n].x;
      cloud->points[i].y = cloud_temp->points[n].y;
      cloud->points[i].z = cloud_temp->points[n].z;
    }
  }
  void predict(){
    for (int i=0; i<SIZE; i++){
      cloud->points[i].x+= rand_normal(0, varience);
      cloud->points[i].y+= rand_normal(0, varience);
      cloud->points[i].z+= rand_normal(0, varience);
    }
  }
  void measure(){
  }
  double likelihood(int i){
    // return (1/ (10 + (cloud->points[i].x-5)*(cloud->points[i].x-5)
    // 	       //*(cloud->points[i].x-5)*(cloud->points[i].x-5) 
    // 	       + cloud->points[i].y*cloud->points[i].y
    // 	       //*cloud->points[i].y*cloud->points[i].y 
    // 		   + cloud->points[i].z*cloud->points[i].z))
    //   //* cloud->points[i].z*cloud->points[i].z );
    //   +(1/ (10 + (cloud->points[i].x+5)*(cloud->points[i].x+5)
    // 	   //*(cloud->points[i].x-5)*(cloud->points[i].x-5) 
    // 	   + cloud->points[i].y*cloud->points[i].y
    // 	   //*cloud->points[i].y*cloud->points[i].y 
    // 	       + cloud->points[i].z*cloud->points[i].z))
    // ;
    //* cloud->points[i].z*cloud->points[i].z ); 
    
    return 2.0 +  1.2*exp(0.05*(-(cloud->points[i].x-5)*(cloud->points[i].x-5)
		//*(cloud->points[i].x-5)*(cloud->points[i].x-5) 
		- cloud->points[i].y*cloud->points[i].y
		//*cloud->points[i].y*cloud->points[i].y 
		- cloud->points[i].z*cloud->points[i].z))
      //* cloud->points[i].z*cloud->points[i].z );
	       +exp (.050*(-(cloud->points[i].x+5)*(cloud->points[i].x+5)
	    //*(cloud->points[i].x-5)*(cloud->points[i].x-5) 
	    - cloud->points[i].y*cloud->points[i].y
	    //*cloud->points[i].y*cloud->points[i].y 
		      - cloud->points[i].z*cloud->points[i].z));
      }
  void weight_calc(){
    double sum_weight=0.0;
    for (int i=0; i<SIZE; i++){
      weight[i]=likelihood(i);
      sum_weight += weight[i];
    }
    ROS_INFO("SUM:%f", likelihood(0));
    for (int i=0; i<SIZE; i++){
      weight[i]/=(sum_weight/SIZE) ;
    }
  }
  void renew(){
    resample();
    predict();
    weight_calc();
  }
  void send_msg(){
    ROS_INFO("start point prob: size=%d", SIZE);
    pcl::toROSMsg(*cloud, p_msg);
    p_msg.header.frame_id = "/eng2/7f/73B2";
    point_pub.publish(p_msg);
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "point_calc");
  
  Point_Class PC;
  ROS_INFO("start point prob: size=%d", SIZE);
  while (ros::ok()){
    PC.renew();
    PC.send_msg();
    ros::spinOnce();
    PC.loop_rate.sleep();
  }
  
  return 0;
}
