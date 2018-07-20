#ifndef INCLUDE_IMAGE_CLASS
#define INCLUDE_IMAGE_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//画像取得用
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>


class beego_control{
	private:
		cv::Mat cur_image;
		cv::Mat pre_image;
	public:

		beego_control();
		bool is_cur_image(void);
		bool is_pre_image(void);
		void subscribe_image(void);
		void set_cur_image(void);
		void set_image(void);
//		void set_debug_image(cv::Mat& temp_image);
		cv::Mat& get_cur_image_by_ref(void);
		cv::Mat& get_pre_image_by_ref(void);
		virtual ~image_class();
		virtual void publish_debug_image(cv::Mat& temp_image);
		virtual void define_variable(void);

	protected:
		bool PROCESS_ONCE;
		ros::NodeHandle nh_pub,nh_sub;
		image_transport::Publisher pub;
		ros::Subscriber sub;
		image_transport::ImageTransport it;
		ros::CallbackQueue queue;
		cv_bridge::CvImagePtr cvbridge_image;

		void set_pre_image(void);
		virtual void image_callback(const sensor_msgs::ImageConstPtr& msg);
};

beego_control::beego_control()
	:it(nh_pub),PROCESS_ONCE(true)
{
	std::cout<<"in image_class constracter\n";
//		  cur_image.reserve();
//		  pre_image.reserve();
//		  debug_image.reserve();

}
bool beego_control::is_cur_image(void){
	return (!cur_image.empty());
}
bool beego_control::is_pre_image(void){
	return (!pre_image.empty());
}
void beego_control::subscribe_image(void){
	queue.callOne(ros::WallDuration(1));
}
void beego_control::set_cur_image(void){
	if(!PROCESS_ONCE)
		cur_image=cvbridge_image->image.clone();
}
void beego_control::set_image(void){
	subscribe_image();
	if(is_cur_image()){
		set_pre_image();
	 }
	set_cur_image();
}

cv::Mat& beego_control::get_cur_image_by_ref(void){
	return cur_image;
}
cv::Mat& beego_control::get_pre_image_by_ref(void){
	return pre_image;
}


beego_control::~beego_control()
{
	cur_image.release();
	pre_image.release();
}

void beego_control::set_pre_image(void){
	pre_image=cur_image.clone();
}


void beego_control::define_variable(void){
	pub=it.advertise("left_image",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/left/image_rect_color",1,&image_class::image_callback,this);
}

void beego_control::image_callback(const sensor_msgs::ImageConstPtr& msg)
{

	try{
		std::cout<<"image_callback \n";
		cvbridge_image=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		PROCESS_ONCE=false;
	}
	catch(cv_bridge::Exception& e) {
		std::cout<<"image_callback Error \n";
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
		return ;
	}
}

void beego_control::publish_debug_image(cv::Mat& temp_image){
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=temp_image.clone();
	pub.publish(publish_cvimage->toImageMsg());

}

#endif
