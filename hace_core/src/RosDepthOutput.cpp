#include "RosDepthOutput.h"

namespace op {

    void RosDepthOutput::init(std::shared_ptr<op::HumanProcessor> hp) {
        hp_ptr_ = std::move(hp);
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
                cv::Mat depth_image = datum.depth_image_;

                // Accesing each element of the keypoints
                const auto& poseKeypoints = datum.poseKeypoints;


                hp_ptr_->processHumans(poseKeypoints, datum.cvOutputData, depth_image, datum.image_time_);
            }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }


}
