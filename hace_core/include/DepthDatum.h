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
        cv::Mat depth_image_ptr_;
        std::string frame_;

        DepthDatum(cv::Mat depth_image = {}, const std::string& frame = "") :
                depth_image_ptr_(depth_image),
                frame_(frame)
        {}

        inline void setDepthImage(cv::Mat ptr) {depth_image_ptr_ = ptr;}
        inline void setFrame(const std::string& frame) {frame_ = frame;}
    };

}

#endif //HACE_CORE_DEPTHDATUM_H
