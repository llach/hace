#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// OpenPose dependencies
#include <openpose/headers.hpp>

#include "cv_convert.h"

namespace op
{

    class OP_API RosImageProducer : public Producer
    {
    public:

        RosImageProducer(const std::string& topic);

        ~RosImageProducer();


        inline bool isOpened() const
        {
            // TODO check whether images are send
            return true;
        }

        std::vector<cv::Mat> getCameraMatrices();

        std::vector<cv::Mat> getCameraExtrinsics();

        std::vector<cv::Mat> getCameraIntrinsics();

        std::string getNextFrameName();

        void set(const int capProperty, const double value);

        double get(const int capProperty);

        void release();

    protected:
        cv::Mat getRawFrame();

        std::vector<cv::Mat> getRawFrames();

    private:

        DELETE_COPY(RosImageProducer);

        void imageCallback(const sensor_msgs::Image::ConstPtr& image);

        long long mFrameNameCounter_;
        cv::Mat mBuffer_;
        std::mutex mBufferMutex_;

        std::shared_ptr<ros::Subscriber> sub_ptr_;

        const std::string topic_;
    };
}