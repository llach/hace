#include "RosWorkerOutput.h"

namespace op {

    void RosWorkerOutput::init(const std::string& output_topic) {
        ros::NodeHandle n("~");

        output_topic_ = output_topic;

        image_pub_ = n.advertise<sensor_msgs::Image>(output_topic_, 1);
    }

    void RosWorkerOutput::initializationOnThread() {};

    void RosWorkerOutput::workConsumer(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
    {
        try
        {
            // process data from openpose given the data is not null
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {

                // only publish if we got subscribers
                if (image_pub_.getNumSubscribers() > 0){
                    // do costly conversion and publish
                    sensor_msgs::Image ros_image = cv_convert::imageFromMat(datumsPtr->at(0).cvOutputData);

                    // TODO set correct frame id
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

}
