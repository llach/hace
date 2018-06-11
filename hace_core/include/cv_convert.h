#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "boost/endian/conversion.hpp"
#include <boost/regex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "openpose/utilities/errorAndLog.hpp"

namespace cv_convert {

    int depthStrToInt(const std::string depth);

    int getCvType(const std::string& encoding);

    // Converts a ROS Image to a cv::Mat by sharing the data or changing its endianness if needed
    cv::Mat matFromImage(const sensor_msgs::Image& source);

    // converts a cv::Mat image to a ROS image
    sensor_msgs::Image imageFromMat(cv::Mat image);
}