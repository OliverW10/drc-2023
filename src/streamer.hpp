#ifndef STREAMER_H
#define STREAMER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace streamer{

void initStreaming();
void imshow(std::string name, const cv::Mat& image);

}
#endif // LOGGING_H