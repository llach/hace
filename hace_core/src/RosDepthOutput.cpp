#include "RosDepthOutput.h"

namespace op {

    void RosDepthOutput::init(const std::string& output_topic,
                              const std::string& people_topic,
                              const std::string& marker_topic,
                              const std::string& camera_info_topic, float min_depth) {
        ros::NodeHandle n("~");

        output_topic_ = output_topic;
        people_topic_ = people_topic;
        marker_topic_ = marker_topic;
        camera_info_topic_ = camera_info_topic;

        min_depth_ = min_depth;

        image_pub_ = n.advertise<sensor_msgs::Image>(output_topic_, 1);
        people_pub_ = n.advertise<hace_msgs::MinimalHumans>(people_topic_, 1);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(camera_frame_, marker_topic_));
        visual_tools_->loadMarkerPub();

        got_camera_info_ = false;
        camera_info_sub_ = n.subscribe(camera_info_topic_, 1, &RosDepthOutput::cameraInfoCallback, this);
    }

    void RosDepthOutput::initializationOnThread() {};

    void RosDepthOutput::workConsumer(const std::shared_ptr<std::vector<op::DepthDatum>>& datumsPtr)
    {
        if(!got_camera_info_) return;

        try
        {

            // process data from openpose given the data is not null
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const auto& datum = datumsPtr->at(0);
                cv::Mat depth_image = datum.depth_image_;

                visual_tools_->deleteAllMarkers();

                // create humans and marker
                hace_msgs::MinimalHumans humans;
                visualization_msgs::MarkerArray human_marker;

                // Accesing each element of the keypoints
                const auto& poseKeypoints = datum.poseKeypoints;

                marker_count_ = 0;

                for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
                {
                    hace_msgs::MinimalHuman human;
                    human.uuid = std::to_string(person);

                    human.face.header.frame_id = camera_frame_;
                    keypointToPose(human.face.pose, poseKeypoints, depth_image, person, 0);

                    human.lhand.header.frame_id = camera_frame_;
                    keypointToPose(human.lhand.pose, poseKeypoints, depth_image, person, 7);

                    human.rhand.header.frame_id = camera_frame_;
                    keypointToPose(human.rhand.pose, poseKeypoints, depth_image, person, 4);

                    if(!std::isnan(human.face.pose.position.x) && !std::isnan(human.face.pose.position.y) && !std::isnan(human.face.pose.position.z)){
                        visual_tools_->publishSphere(human.face.pose, rviz_visual_tools::WHITE, rviz_visual_tools::scales::XXLARGE);
                    }

                    if(!std::isnan(human.lhand.pose.position.x) && !std::isnan(human.lhand.pose.position.y) && !std::isnan(human.lhand.pose.position.z)){
                        visual_tools_->publishSphere(human.lhand.pose, rviz_visual_tools::BLACK, rviz_visual_tools::scales::XLARGE);
                    }

                    if(!std::isnan(human.rhand.pose.position.x) && !std::isnan(human.rhand.pose.position.y) && !std::isnan(human.rhand.pose.position.z)){
                        visual_tools_->publishSphere(human.rhand.pose, rviz_visual_tools::BLACK, rviz_visual_tools::scales::XLARGE);
                    }

                    humans.humans.push_back(human);
                }

                previous_humans_ = humans;

                if(!humans.humans.empty()) people_pub_.publish(humans);
                visual_tools_->trigger();


                // only publish if we got subscribers
                if (image_pub_.getNumSubscribers() > 0){
                    // do costly conversion and publish
                    sensor_msgs::Image ros_image = cv_convert::imageFromMat(datum.cvOutputData);
                    ros_image.header.frame_id = camera_frame_;

                    image_pub_.publish(ros_image);
                }

            }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    float RosDepthOutput::getValueAroundPoint(cv::Mat image, int x, int y, int radius){

        std::vector<double> v;

        if(image.empty() || x == 0.0 || y == 0.0){
            return std::numeric_limits<float>::quiet_NaN();
        }

        for(int i = -radius; i <= radius; i++){
            for(int j = -radius; j <= radius; j++) {
                    float vi = image.at<float>(std::min(std::max(0,y+j), image.rows-1), std::min(std::max(0, x+i), image.cols-1));

                    if(std::isinf(vi) || std::isnan(vi)) {
                        continue;
                    }

                    v.push_back(vi);
            }
        }

        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        if(mean < min_depth_) {
            return std::numeric_limits<float>::quiet_NaN();
        } else {
            return mean;
        }

    }

    void RosDepthOutput::keypointToPose(geometry_msgs::Pose& pose, const op::Array<float>& keypoints,
                                        cv::Mat& depth_image, int pind, int kind){

        int pixel_x = keypoints[{pind, kind, 0}];
        int pixel_y = keypoints[{pind, kind, 1}];

        float z = getValueAroundPoint(depth_image, pixel_x, pixel_y);

        if(pixel_x == 0.0 || pixel_y == 0.0 || std::isnan(z)){
            pose.position.x = std::numeric_limits<float>::quiet_NaN();
            pose.position.y = std::numeric_limits<float>::quiet_NaN();
            pose.position.z = std::numeric_limits<float>::quiet_NaN();
        } else {
            cv::Point2d kpoint(pixel_x, pixel_y);
            cv::Point3d kpoint3d = camera_model_.projectPixelTo3dRay(camera_model_.rectifyPoint(kpoint));

            pose.position.x = kpoint3d.x;
            pose.position.y = kpoint3d.y;
            pose.position.z = z;

            pose.orientation.w = 1.0;
        }

    }

    void RosDepthOutput::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr camera_info){

        if(got_camera_info_) return;

        op::log("Got camera info from " + camera_info_topic_);

        camera_model_.fromCameraInfo(camera_info);
        camera_frame_ = camera_info->header.frame_id;

        visual_tools_->setBaseFrame(camera_frame_);

        got_camera_info_ = true;
    }

}
