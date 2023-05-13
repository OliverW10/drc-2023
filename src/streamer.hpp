#ifndef STREAMER_H
#define STREAMER_H

#include <string>
#include <opencv2/opencv.hpp>

// streams images and other debug info
class Streamer{
public:
    // get singleton instance
    static Streamer& getInstance();
    static void imshow(std::string name, const cv::Mat& image);
private:
    void showImage(std::string name, const cv::Mat& image);
    Streamer();
};

#endif // LOGGING_H