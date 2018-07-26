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
    {   cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // Get frame
            //cv::Mat cvMat = cv_convert::matFromImage(*image);
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

            // convert to bgr TODO automatically select correct conversion instead of hardcoding
            //cv::cvtColor(cvMat, cvMat, cv::COLOR_RGB2BGR);

            // Move to buffer
            if (!cv_ptr->image.empty())
            {
                const std::lock_guard<std::mutex> lock{rgbBufferMutex_};
                std::swap(rgbBuffer_, cv_ptr->image);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void RosDepthInput::depthCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv::Mat depth_mat;
            // Get frame
            //cv::Mat cvMat;// = cv_convert::matFromImage(*image);
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
            //cv::cvtColor(cvMat, cvMat, cv::COLOR_RGB2BGR);

            cv_ptr->image.convertTo(depth_mat, CV_32F, 0.001);
            
            // Move to buffer
            if (!cv_ptr->image.empty())
            {
                const std::lock_guard<std::mutex> lock{depthBufferMutex_};
                std::swap(depthBuffer_, depth_mat);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
