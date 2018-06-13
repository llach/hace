#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// OpenPose dependencies
#include <openpose/headers.hpp>

#include "DepthDatum.h"
#include "cv_convert.h"

namespace op
{

    class RosDepthInput : public op::WorkerProducer<std::shared_ptr<std::vector<DepthDatum>>>
    {
        public:

            void init(const std::string& rgb_topic,
                          const std::string& depth_topic);

            inline void initializationOnThread() {};

            std::shared_ptr<std::vector<op::DepthDatum>> workProducer();

        private:

            void depthCallback(const sensor_msgs::Image::ConstPtr& image);
            void rgbCallback(const sensor_msgs::Image::ConstPtr& image);

            std::string rgb_topic_;
            std::string depth_topic_;

            cv::Mat rgbBuffer_;
            std::mutex rgbBufferMutex_;

            cv::Mat depthBuffer_;
            std::mutex depthBufferMutex_;

            std::shared_ptr<ros::Subscriber> rgbsub_ptr_;
            std::shared_ptr<ros::Subscriber> depthsub_ptr_;


    };
}