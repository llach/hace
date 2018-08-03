//
// Created by llach on 11.06.18.
//

#ifndef HACE_CORE_DEPTHDATUM_H
#define HACE_CORE_DEPTHDATUM_H

// If the user needs his own variables, he can inherit the op::Datum struct and add them
// UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define Wrapper<UserDatum> instead of
// Wrapper<op::Datum>

namespace op
{
    struct DepthDatum : public Datum
    {
        cv::Mat depth_image_;
        std::string frame_;
        ros::Time image_time_;

        DepthDatum(cv::Mat depth_image = {}, const std::string& frame = "", ros::Time image_time = ros::Time()) :
                depth_image_(depth_image),
                frame_(frame),
                image_time_(image_time)
        {}

        inline void setDepthImage(cv::Mat image) {depth_image_ = image;}
        inline void setFrame(const std::string& frame) {frame_ = frame;}
        inline void setImageTime(const ros::Time image_time) {image_time_ = image_time;}
    };

}

#endif //HACE_CORE_DEPTHDATUM_H
