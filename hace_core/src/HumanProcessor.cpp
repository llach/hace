#include "HumanProcessor.h"
#include <cv_bridge/cv_bridge.h>

namespace op {

    HumanProcessor::HumanProcessor(const std::string& output_topic,
                                   const std::string& people_topic,
                                   const std::string& marker_topic,
                                   const std::string& camera_info_topic,
                                   float min_depth, float human_thresh) {
        ros::NodeHandle n("~");

        output_topic_ = output_topic;
        people_topic_ = people_topic;
        marker_topic_ = marker_topic;
        camera_info_topic_ = camera_info_topic;

        min_depth_ = min_depth;

        image_pub_ = n.advertise<sensor_msgs::Image>(output_topic_, 1);
        people_pub_ = n.advertise<hace_msgs::MinimalHumans>(people_topic_, 1);
        debug_pub_ = n.advertise<hace_msgs::HaceDebug>("/hace/debug", 1);

        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(camera_frame_, marker_topic_));
        visual_tools_->loadMarkerPub();

        got_camera_info_ = false;
        camera_info_sub_ = n.subscribe(camera_info_topic_, 1, &HumanProcessor::cameraInfoCallback, this);

        id_counter_ = 0;
        human_thresh_ = human_thresh;
    }

    void HumanProcessor::processHumans(Array<float> keypoints, const cv::Mat image, cv::Mat& depth_image)
    {
        if(!got_camera_info_) return;

        if(sent_humans_) visual_tools_->deleteAllMarkers();
        sent_humans_ = false;

        //assign uuids
        trackHumans(keypoints);

        // create humans and marker
        hace_msgs::MinimalHumans humans;
        visualization_msgs::MarkerArray human_marker;

        int marker_count = 0;

        for (auto person = 0 ; person < keypoints.getSize(0) ; person++)
        {
            hace_msgs::MinimalHuman human;
            human.uuid = uuids[person];

            human.face.header.frame_id = camera_frame_;
            keypointToPose(human.face.pose, keypoints, depth_image, person, 1);

            human.lhand.header.frame_id = camera_frame_;
            keypointToPose(human.lhand.pose, keypoints, depth_image, person, 7);

            human.rhand.header.frame_id = camera_frame_;
            keypointToPose(human.rhand.pose, keypoints, depth_image, person, 4);

            if(!checkPoseNan(human.face.pose)){
                std::cout << "got face" << human.face.pose << std::endl;
                visual_tools_->publishSphere(human.face.pose, rviz_visual_tools::RED, rviz_visual_tools::scales::XXXLARGE);
                marker_count++;
            }

            if(!checkPoseNan(human.lhand.pose)){
                visual_tools_->publishSphere(human.lhand.pose, rviz_visual_tools::BLACK, rviz_visual_tools::scales::XLARGE);
                marker_count++;
            }

            if(!checkPoseNan(human.rhand.pose)){
                visual_tools_->publishSphere(human.rhand.pose, rviz_visual_tools::BLACK, rviz_visual_tools::scales::XLARGE);
                marker_count++;
            }

            if(!checkHumanNan(human)) humans.humans.push_back(human);

            cv::putText(image, human.uuid, getHighestPoint(keypoints, person), cv::FONT_HERSHEY_DUPLEX,
                        1.6,  cvScalar(213, 38, 181));

        }

        previous_humans_ = humans;

        if(!humans.humans.empty()) {
            sent_humans_ = true;
            people_pub_.publish(humans);
        }

        visual_tools_->trigger();

        if(debug_){
            hace_msgs::HaceDebug dm;

            dm.numPeople = keypoints.getSize(0);
            dm.numPeopleMarker = marker_count;
            dm.numValidPeople = humans.humans.size();

            debug_pub_.publish(dm);
        }

        // only publish if we got subscribers
        if (image_pub_.getNumSubscribers() > 0){
            // do costly conversion and publish
            
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg; // >> message to be sent

std_msgs::Header header; // empty header
//header.seq = counter; // TODO: user defined counter
header.stamp = ros::Time::now(); // time
            header.frame_id = camera_frame_;

img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
img_bridge.toImageMsg(img_msg);
            
            

            image_pub_.publish(img_msg);
        }

    }

    void HumanProcessor::trackHumans(Array<float>& keypoints){

        std::vector<std::string> new_ids;

        // matrix t-1 humans X t humans
        float dist_mat[prev_keypoints_.getSize(0)][keypoints.getSize(0)];

        if(!prev_keypoints_.empty()) {

            // calculate distance matrix
            for (int i = 0; i < prev_keypoints_.getSize(0); i++){
                for (int j = 0; j < keypoints.getSize(0); j++){
                    dist_mat[i][j] = distHuman(prev_keypoints_, keypoints, i, j);
                }
            }

            // new uuids
            for(int o = 0; o < keypoints.getSize(0); o++){

                int min_idx;
                float min_dist = std::numeric_limits<float>::infinity();

                for (int k = 0; k < prev_keypoints_.getSize(0); k++){
                    if(dist_mat[k][o] < min_dist){
                        min_dist = dist_mat[k][o];
                        min_idx = k;
                    }
                }

                if (min_dist == std::numeric_limits<float>::infinity()) {
                    new_ids.push_back(std::to_string(id_counter_));
                    id_counter_++;
                    continue;
                }

                bool found_min = true;

                for(int k = 0; k < keypoints.getSize(0); k++){
                    if(dist_mat[min_idx][k] < min_dist) found_min = false;
                }

                if(found_min && min_dist < human_thresh_){
                    new_ids.push_back(uuids.at(min_idx));
                } else {
                    new_ids.push_back(std::to_string(id_counter_));
                    id_counter_++;
                }

            }


        } else {

            for (int i = 0; i < keypoints.getSize(0); i++){
                new_ids.push_back(std::to_string(id_counter_));
                id_counter_++;
            }
        }

        if(superdebug_){

            for (int i = 0; i < prev_keypoints_.getSize(0); i++){
                std::cout << "| ";
                for (int j = 0; j < keypoints.getSize(0); j++){
                    std::cout << dist_mat[i][j] << " ";
                }
                std::cout << "|" << std::endl;
            }

            std::cout << std::endl;
        }

        // store new data
        prev_keypoints_ = keypoints;
        uuids = new_ids;

    }

    float HumanProcessor::distHuman(Array<float> l1, Array<float> l2, int p1, int p2){
        float dist = 0;
        int matched_keypoints = 0;

        for (int i = 0; i < l1.getSize(1); i++){
            float p1x = l1[{p1, i, 0}];
            float p1y = l1[{p1, i, 1}];

            float p2x = l2[{p2, i, 0}];
            float p2y = l2[{p2, i, 1}];

            // skip this keypoint pair if op did not recognize it in both timesteps
            if(p1x <= 0.0 || p1y <= 0.0 || p2x <= 0.0 || p2y <= 0.0) continue;

            dist += sqrt(pow(p1x - p2x, 2) +  pow(p1y - p2y, 2));
            matched_keypoints++;
        }

        dist /= matched_keypoints;

        return dist > 0 ? dist : std::numeric_limits<float>::infinity();
    }


    bool HumanProcessor::checkHumanNan(const hace_msgs::MinimalHuman& human){
        return checkPoseNan(human.face.pose) && checkPoseNan(human.lhand.pose) && checkPoseNan(human.rhand.pose);
    }

    bool HumanProcessor::checkPoseNan(const geometry_msgs::Pose& pose){
        return std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z);
    }

    void HumanProcessor::removeOutlier(std::vector<float>& v, float& mean){

        std::vector<double> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });

        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size());

        std::vector<float> deleted_elements;

        v.erase(std::remove_if(
                v.begin(), v.end(),
                [&](const float& x) {
                    bool c = x > (mean + stdev);
                    if (c) deleted_elements.push_back(x);
                    return c;
                }), v.end());

        if(!deleted_elements.empty()){
            std::cout << "Removed" << deleted_elements << " from " << v << std::endl;

            double sum = std::accumulate(v.begin(), v.end(), 0.0);
            mean = sum / v.size();
        }

    }

    float HumanProcessor::getValueAroundPoint(cv::Mat image, int x, int y, int radius){

        std::vector<float> v;

        if(image.empty() || x == 0.0 || y == 0.0){
            return std::numeric_limits<float>::quiet_NaN();
        }

        for(int i = -radius; i <= radius; i++){
            for(int j = -radius; j <= radius; j++) {
                float vi = image.at<float>(std::min(std::max(0,y+j), image.rows-1), std::min(std::max(0, x+i), image.cols-1));
                std::cout << vi << ", ";

                if(std::isinf(vi) || std::isnan(vi)) {
                    continue;
                }

                v.push_back(vi);
            }
        }
        std::cout << std::endl;

        if(v.empty()) {
            return std::numeric_limits<float>::quiet_NaN();
        }

        float med = median(v);

        if(med < min_depth_) {
            return std::numeric_limits<float>::quiet_NaN();
        } else {
            return med;
        }

    }

    cv::Point2d HumanProcessor::getHighestPoint(const op::Array<float>& keypoints, int pind){

        cv::Point2d p(9000,9000);

        for (int i = 0; i < keypoints.getSize(1); i++){
            int px = keypoints[{pind, i, 0}];
            int py = keypoints[{pind, i, 1}];

            if(p.y > py && py > 0.0){
                p.x = px;
                p.y = py;
            }
        }

        return p;

    }

    void HumanProcessor::keypointToPose(geometry_msgs::Pose& pose, const op::Array<float>& keypoints,
                                        cv::Mat& depth_image, int pind, int kind){

        int pixel_x = keypoints[{pind, kind, 0}];
        int pixel_y = keypoints[{pind, kind, 1}];

        float z = getValueAroundPoint(depth_image, pixel_x, pixel_y);
        
                    std::cout << "x, y" << pixel_x << ", " << pixel_y << std::endl;


        if(pixel_x == 0.0 || pixel_y == 0.0 || std::isnan(z)){
            std::cout << "oh no" << std::endl;
            pose.position.x = std::numeric_limits<float>::quiet_NaN();
            pose.position.y = std::numeric_limits<float>::quiet_NaN();
            pose.position.z = std::numeric_limits<float>::quiet_NaN();
        } else {
            cv::Point2d kpoint(pixel_x, pixel_y);
            cv::Point3d kpoint3d = camera_model_.projectPixelTo3dRay(camera_model_.rectifyPoint(kpoint));

            pose.position.x = kpoint3d.x;
            pose.position.y = kpoint3d.y;
            pose.position.z = z;

            std::cout << "oh YES" << std::endl;
            pose.orientation.w = 1.0;
        }

    }

    cv::Point2d HumanProcessor::com(const op::Array<float> &keypoints, std::vector<int> kp_ids) {

    }

    void HumanProcessor::keypointsToPose(geometry_msgs::Pose &pose, const op::Array<float> &keypoints,
                                         cv::Mat &depth_image, int pind, std::vector<int> kp_ids){

        auto p = com(keypoints, kp_ids);

        float z = getValueAroundPoint(depth_image, p.x, p.y);

        if(p.x == 0.0 || p.y == 0.0 || std::isnan(z)){
            pose.position.x = std::numeric_limits<float>::quiet_NaN();
            pose.position.y = std::numeric_limits<float>::quiet_NaN();
            pose.position.z = std::numeric_limits<float>::quiet_NaN();
        } else {
            cv::Point3d kpoint3d = camera_model_.projectPixelTo3dRay(camera_model_.rectifyPoint(p));

            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = z;

            pose.orientation.w = 1.0;
        }

    }

    void HumanProcessor::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr camera_info){

        if(got_camera_info_) return;

        op::log("Got camera info from " + camera_info_topic_);

        camera_model_.fromCameraInfo(camera_info);
        camera_frame_ = camera_info->header.frame_id;

        visual_tools_->setBaseFrame(camera_frame_);

        got_camera_info_ = true;
    }

}
