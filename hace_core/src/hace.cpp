#include "ros/ros.h"
#include "sensor_msgs/Image.h"


void rgbCallback(const sensor_msgs::Image::ConstPtr& image){}
void depthCallback(const sensor_msgs::Image::ConstPtr& image){}


int main (int argc, char** argv){
    ros::init(argc, argv, "hace");
    ros::NodeHandle n("~");

    std::string rgb_topic, depth_topic;
    bool output;

    n.param("rgb_topic", rgb_topic, std::string("/camera/rgb/image_raw"));
    n.param("depth_topic", depth_topic, std::string("/camera/depth_registered/image_raw"));
    n.param("output_image", output, false);

    ros::Subscriber rgb_sub = n.subscribe(rgb_topic, 10, rgbCallback);
    ros::Subscriber depth_sub = n.subscribe(depth_topic, 10, depthCallback);
    
    ROS_INFO("... and we're spinning in the main thread!");
    ros::MultiThreadedSpinner().spin();
}