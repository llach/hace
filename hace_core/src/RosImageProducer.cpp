#include <RosImageProducer.h>

namespace op
{
    RosImageProducer::RosImageProducer(const std::string& topic) :
            Producer{ProducerType::Webcam},
            topic_{topic}
    {
        try
        {
            ros::NodeHandle n("~");
            sub_ptr_.reset(new ros::Subscriber(n.subscribe(topic_, 1, &RosImageProducer::imageCallback, this)));
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }


    RosImageProducer::~RosImageProducer() {}

    cv::Mat RosImageProducer::getRawFrame()
    {
        try
        {
            mFrameNameCounter_++; // Simple counter: 0,1,2,3,...

            // Retrieve frame from buffer
            cv::Mat cvMat;
            auto cvMatRetrieved = false;
            while (!cvMatRetrieved)
            {
                // Retrieve frame
                std::unique_lock<std::mutex> lock{mBufferMutex_};
                if (!mBuffer_.empty())
                {
                    std::swap(cvMat, mBuffer_);
                    cvMatRetrieved = true;
                }
                    // No frames available -> sleep & wait
                else
                {
                    lock.unlock();
                    std::this_thread::sleep_for(std::chrono::microseconds{5});
                }
            }
            return cvMat;

            // Naive implementation - No flashing buffers
            // return VideoCaptureReader::getRawFrame();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return cv::Mat();
        }
    }


    std::vector<cv::Mat> RosImageProducer::getRawFrames()
    {
        try
        {
            return std::vector<cv::Mat>{getRawFrame()};
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return {};
        }
    }

    void RosImageProducer::imageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
        try
        {
            // Get frame
            cv::Mat cvMat;// = cv_convert::matFromImage(*image);

            // convert to bgr TODO automatically select correct conversion instead of hardcoding
            //cv::cvtColor(cvMat, cvMat, cv::COLOR_RGB2BGR);

            // Move to buffer
            if (!cvMat.empty())
            {
                const std::lock_guard<std::mutex> lock{mBufferMutex_};
                std::swap(mBuffer_, cvMat);
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    std::string RosImageProducer::getNextFrameName()
    {
        try
        {
            return std::to_string(mFrameNameCounter_);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return "";
        }
    }

    void RosImageProducer::release() {}

    void RosImageProducer::set(const int capProperty, const double value){
        try
        {
            if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
                log("This property is read-only.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
            else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
                log("This property is read-only.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
            else if (capProperty == CV_CAP_PROP_POS_FRAMES)
                log("This property is read-only.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
            else if (capProperty == CV_CAP_PROP_FRAME_COUNT || capProperty == CV_CAP_PROP_FPS)
                log("This property is read-only.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
            else
                log("Unknown property.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    double RosImageProducer::get(const int capProperty)
    {
        try
        {
            // TODO read frame dimesions from each frame
            if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
            {
                return 640.;
            }
            else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
            {
               return 480.;
            }
            else if (capProperty == CV_CAP_PROP_POS_FRAMES)
                return (double)mFrameNameCounter_;
            else if (capProperty == CV_CAP_PROP_FRAME_COUNT)
                return -1.;
            else if (capProperty == CV_CAP_PROP_FPS)
                return -1.;
            else
            {
                log("Unknown property.", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
                return -1.;
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return 0.;
        }
    }

    std::vector<cv::Mat> RosImageProducer::getCameraMatrices()
    {
        return {};
    }

    std::vector<cv::Mat> RosImageProducer::getCameraExtrinsics()
    {
        return {};
    }

    std::vector<cv::Mat> RosImageProducer::getCameraIntrinsics()
    {
        return {};
    }

}