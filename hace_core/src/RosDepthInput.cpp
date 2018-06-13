#include "RosDepthInput.h"

namespace op {

    void RosDepthInput::init(const std::string& rgb_topic,
                        const std::string& depth_topic)
    {
        ros::NodeHandle n("~");

        rgb_topic_ = rgb_topic;
        depth_topic_ = depth_topic;

        rgbsub_ptr_.reset(new ros::Subscriber(n.subscribe(rgb_topic_, 1, &RosDepthInput::rgbCallback, this)));
        depthsub_ptr_.reset(new ros::Subscriber(n.subscribe(depth_topic_, 1, &RosDepthInput::depthCallback, this)));
    }

    std::shared_ptr<std::vector<op::DepthDatum>> RosDepthInput::workProducer()
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<DepthDatum>>();

        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        if(!depthBuffer_.empty())
        {
            const std::lock_guard<std::mutex> lock{depthBufferMutex_};
            datum.setDepthImage(depthBuffer_);
        }

        // Move to buffer
        if (!rgbBuffer_.empty())
        {
            const std::lock_guard<std::mutex> lock{rgbBufferMutex_};
            datum.cvInputData = rgbBuffer_;
        } else {
            return nullptr;
        }

        return datumsPtr;

    }

    void RosDepthInput::rgbCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        try
        {
            // Get frame
            cv::Mat cvMat = cv_convert::matFromImage(*image);

            // convert to bgr TODO automatically select correct conversion instead of hardcoding
            cv::cvtColor(cvMat, cvMat, cv::COLOR_RGB2BGR);

            // Move to buffer
            if (!cvMat.empty())
            {
                const std::lock_guard<std::mutex> lock{rgbBufferMutex_};
                std::swap(rgbBuffer_, cvMat);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void RosDepthInput::depthCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        try
        {
            // Get frame
            cv::Mat cvMat = cv_convert::matFromImage(*image);

            // Move to buffer
            if (!cvMat.empty())
            {
                const std::lock_guard<std::mutex> lock{depthBufferMutex_};
                std::swap(depthBuffer_, cvMat);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
