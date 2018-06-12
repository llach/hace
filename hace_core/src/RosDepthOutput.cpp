#include "RosDepthOutput.h"

namespace op {

    void RosDepthOutput::init(const std::string& output_topic,
                              const std::string& people_topic,
                              const std::string& marker_topic,
                              const std::string& camera_info_topic) {
        ros::NodeHandle n("~");

        output_topic_ = output_topic;
        people_topic_ = people_topic;
        marker_topic_ = marker_topic;
        camera_info_topic_ = camera_info_topic;

        image_pub_ = n.advertise<sensor_msgs::Image>(output_topic_, 1);
        people_pub_ = n.advertise<hace_msgs::MinimalHumans>(people_topic_, 1);
        marker_pub_ = n.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

        got_camera_info_ = false;
        camera_info_sub_ = n.subscribe(camera_info_topic_, 1, &RosDepthOutput::cameraInfoCallback, this);
    }

    void RosDepthOutput::initializationOnThread() {};

    void RosDepthOutput::workConsumer(const std::shared_ptr<std::vector<op::DepthDatum>>& datumsPtr)
    {
        try
        {
            // process data from openpose given the data is not null
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const auto& datum = datumsPtr->at(0);
                cv::Mat depth_image = datum.depth_image_ptr_;

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

                    const auto& keypoint_marker =  produceHumanMarker(person, human);
                    if(!keypoint_marker.empty()) human_marker.markers.insert(human_marker.markers.end(), keypoint_marker.begin(), keypoint_marker.end());

                    humans.humans.push_back(human);
                }

                if(!humans.humans.empty()) people_pub_.publish(humans);
                if(!human_marker.markers.empty()) marker_pub_.publish(human_marker);

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

        float sum = 0.0;

        if(image.empty() || std::isnan(x) || std::isnan(y)){
            return std::numeric_limits<float>::quiet_NaN();
        }

        for(int i = -radius; i <= radius; i++){
            for(int j = -radius; j <= radius; j++) {
                    float v = image.at<float>(std::min(std::max(0,y+j), image.rows-1), std::min(std::max(0, x+i), image.cols-1));

                    if(std::isinf(v) || std::isnan(v)) {
                        continue;
                    }
                    sum += v;
            }
        }

        if(std::isinf(sum) || std::isnan(sum)) {
            return std::numeric_limits<float>::quiet_NaN();
        } else {
            return sum / 9;
        }

    }

    void RosDepthOutput::keypointToPose(geometry_msgs::Pose& pose, const op::Array<float>& keypoints,
                                        cv::Mat& depth_image, int pind, int kind){

        int pixel_x = keypoints[{pind, kind, 0}];
        int pixel_y = keypoints[{pind, kind, 1}];

        if(pixel_x == 0 || pixel_y == 0 || !got_camera_info_){
            pose.position.x = std::numeric_limits<float>::quiet_NaN();
            pose.position.y = std::numeric_limits<float>::quiet_NaN();
            pose.position.z = std::numeric_limits<float>::quiet_NaN();
        } else {
            cv::Point2d kpoint(pixel_x, pixel_y);
            cv::Point3d kpoint3d = camera_model_.projectPixelTo3dRay(camera_model_.rectifyPoint(kpoint));

            pose.position.x = kpoint3d.x;
            pose.position.y = kpoint3d.y;
            pose.position.z = getValueAroundPoint(depth_image, pixel_x, pixel_y);

            pose.orientation.w = 1.0;
        }

    }

    std::vector<visualization_msgs::Marker> RosDepthOutput::produceHumanMarker(int pind, hace_msgs::MinimalHuman& human){

        visualization_msgs::Marker face;
        visualization_msgs::Marker lhand;
        visualization_msgs::Marker rhand;

        face.header.frame_id = camera_frame_;
        lhand.header.frame_id = camera_frame_;
        rhand.header.frame_id = camera_frame_;

        face.pose = human.face.pose;
        lhand.pose = human.lhand.pose;
        rhand.pose = human.rhand.pose;

        face.text = std::to_string(pind);
        lhand.text = std::to_string(pind);
        rhand.text = std::to_string(pind);

        face.type = visualization_msgs::Marker::SPHERE;
        lhand.type = visualization_msgs::Marker::SPHERE;
        rhand.type = visualization_msgs::Marker::SPHERE;

        face.scale.x = 0.1;
        face.scale.y = 0.1;
        face.scale.z = 0.1;
        face.color.a = 0.1;
        face.color.r = 1.0;
        face.color.g = 1.0;
        face.color.b = 1.0;

        lhand.scale.x = 0.08;
        lhand.scale.y = 0.08;
        lhand.scale.z = 0.08;
        lhand.color.a = 1.0;
        lhand.color.r = 0.0;
        lhand.color.g = 0.0;
        lhand.color.b = 0.0;

        rhand.scale.x = 0.08;
        rhand.scale.y = 0.08;
        rhand.scale.z = 0.08;
        rhand.color.a = 1.0;
        rhand.color.r = 0.0;
        rhand.color.g = 0.0;
        rhand.color.b = 0.0;

        face.id = marker_count_;
        lhand.id = marker_count_+1;
        rhand.id = marker_count_+2;

        marker_count_ += 3;

        std::vector<visualization_msgs::Marker> marker_array;

        if(!std::isnan(face.pose.position.x)) marker_array.push_back(face);
        if(!std::isnan(lhand.pose.position.x)) marker_array.push_back(lhand);
        if(!std::isnan(rhand.pose.position.x)) marker_array.push_back(rhand);

        return marker_array;
    }

    void RosDepthOutput::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr camera_info){

        if(got_camera_info_) return;

        camera_model_.fromCameraInfo(camera_info);
        camera_frame_ = camera_info->header.frame_id;

        got_camera_info_ = true;
    }

}
