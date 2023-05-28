#ifndef STREAMER_H
#define STREAMER_H

#include <string>
#include <opencv2/core/mat.hpp>

namespace streamer{

void initStreaming();
void imshow(std::string name, const cv::Mat& image);
void closeThread();

}
#endif // LOGGING_H