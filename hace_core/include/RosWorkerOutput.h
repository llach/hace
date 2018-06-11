#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// OpenPose dependencies
#include <openpose/headers.hpp>

#include "cv_convert.h"

namespace op
{

    class RosWorkerOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<op::Datum>>>
    {
    public:

        void init(const std::string& output_topic);

        void initializationOnThread();

        void workConsumer(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

    private:

        ros::Publisher image_pub_;

        std::string output_topic_;

    };
}