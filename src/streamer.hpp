#ifndef STREAMER_H
#define STREAMER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace streamer{

void initStreaming();
void imshow(const std::string& name, const cv::Mat& image, bool copy_immediately = false);
void wait_for_threads();

}
#endif // LOGGING_H