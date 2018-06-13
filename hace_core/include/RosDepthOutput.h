#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_geometry/pinhole_camera_model.h"
#include "visualization_msgs/MarkerArray.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

// OpenPose dependencies
#include <openpose/headers.hpp>
#include "hace_msgs/MinimalHumans.h"

#include "cv_convert.h"
#include "DepthDatum.h"

namespace op
{

    class RosDepthOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<op::DepthDatum>>>
{
    public:

        void init(const std::string& output_topic,
                  const std::string& people_topic,
                  const std::string& marker_topic,
                  const std::string& camera_info_topic, float min_depth = 0.6);

        void initializationOnThread();

        void workConsumer(const std::shared_ptr<std::vector<op::DepthDatum>>& datumsPtr);

    private:

        bool checkHumanNan(const hace_msgs::MinimalHuman& human);
        bool checkPoseNan(const geometry_msgs::Pose& pose);
        float getValueAroundPoint(cv::Mat image, int x, int y, int radius = 1);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr camera_info);
        void keypointToPose(geometry_msgs::Pose& pose, const op::Array<float>& keypoints,
                            cv::Mat& depth_image, int pind, int kind);

        ros::Publisher image_pub_;
        ros::Publisher people_pub_;
        ros::Publisher marker_pub_;
        ros::Subscriber camera_info_sub_;

        std::string output_topic_;
        std::string people_topic_;
        std::string marker_topic_;
        std::string camera_info_topic_;

        int marker_count_;
        float min_depth_;
        bool sent_humans_;

        std::string camera_frame_;
        std::atomic<bool> got_camera_info_;
        image_geometry::PinholeCameraModel camera_model_;

        hace_msgs::MinimalHumans previous_humans_;
        visualization_msgs::MarkerArray previous_human_marker_;

        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};
}