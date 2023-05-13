#include "streamer.hpp"
#include <nadjieb/mjpeg_streamer.hpp>
namespace streamer{

static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};

static nadjieb::MJPEGStreamer streamer;

void initStreaming(){
    streamer.start(8080);
}

void imshow(std::string name, const cv::Mat& image){
    std::vector<uchar> buff;
    cv::imencode(".jpg", image, buff, params);
    streamer.publish("/img", std::string(buff.begin(), buff.end()));
}
}