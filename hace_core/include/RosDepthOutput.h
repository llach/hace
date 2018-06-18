#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_geometry/pinhole_camera_model.h"
#include "visualization_msgs/MarkerArray.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

// OpenPose dependencies
#include <openpose/headers.hpp>
#include "hace_msgs/MinimalHumans.h"
#include "hace_msgs/HaceDebug.h"

#include "cv_convert.h"
#include "DepthDatum.h"
#include "HumanProcessor.h"

namespace op
{

    class RosDepthOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<op::DepthDatum>>>
{
    public:

        void init(std::shared_ptr<op::HumanProcessor> hp);
        void initializationOnThread();
        void workConsumer(const std::shared_ptr<std::vector<op::DepthDatum>>& datumsPtr);

    private:

        std::shared_ptr<op::HumanProcessor> hp_ptr_;

};

}