#ifndef HACE_CORE_HUMANPROCESSOR_H
#define HACE_CORE_HUMANPROCESSOR_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_geometry/pinhole_camera_model.h"

// OpenPose dependencies
#include <openpose/headers.hpp>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "hace_msgs/MinimalHumans.h"
#include "hace_msgs/HaceDebug.h"

#include "cv_convert.h"
#include "DepthDatum.h"

namespace op
{

    class HumanProcessor 
{
    public:

    HumanProcessor(const std::string& output_topic,
                   const std::string& people_topic,
                   const std::string& marker_topic,
                   const std::string& camera_info_topic,
                   float min_depth = 0.6,
                   float human_thresh = 100);

    void processHumans(Array<float> keypoints, const cv::Mat image, cv::Mat& depth_image);

    void trackHumans(Array<float>& keypoints);

    inline bool setDebug(bool debug = true) {debug_ = debug;}

    private:

        cv::Point2d getHighestPoint(const op::Array<float>& keypoints, int pind);
        void removeOutlier(std::vector<float>& v, float& mean);
        bool checkHumanNan(const hace_msgs::MinimalHuman& human);
        bool checkPoseNan(const geometry_msgs::Pose& pose);
        float getValueAroundPoint(cv::Mat image, int x, int y, int radius = 1);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr camera_info);
        float distHuman(Array<float> l1, Array<float> l2, int p1, int p2);
        void keypointToPose(geometry_msgs::Pose& pose, const op::Array<float>& keypoints,
                            cv::Mat& depth_image, int pind, int kind);

        ros::Publisher image_pub_;
        ros::Publisher people_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher debug_pub_;
        ros::Subscriber camera_info_sub_;

        std::string output_topic_;
        std::string people_topic_;
        std::string marker_topic_;
        std::string camera_info_topic_;

        float min_depth_;
        long id_counter_;
        bool sent_humans_;
        bool debug_ = false;
        bool superdebug_ = false;

        float human_thresh_;
        Array<float> prev_keypoints_;
        std::vector<std::string> uuids;

        std::string camera_frame_;
        std::atomic<bool> got_camera_info_;
        image_geometry::PinholeCameraModel camera_model_;

        hace_msgs::MinimalHumans previous_humans_;
        visualization_msgs::MarkerArray previous_human_marker_;

        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    template <typename T>
    T median(std::vector<T> v){
        auto begin = v.begin();
        auto end = v.end();
        const auto size  = std::distance(begin, end);
        std::nth_element(begin, begin + size / 2, end);
        return *std::next(begin, size /2);
    }

};

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& s) {
    os << "[";
    bool first = true;
    for (const auto& item : s) {
        if (!first)
            os << ", ";
        os << item;
        first = false;
    }
    os << "]";
    return os;
}
}


#endif //HACE_CORE_HUMANPROCESSOR_H
