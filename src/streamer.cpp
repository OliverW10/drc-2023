#include "streamer.hpp"
#include <nadjieb/mjpeg_streamer.hpp>
namespace streamer{

static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};

static nadjieb::MJPEGStreamer streamer;

void initStreaming(){
    streamer.start(8080);
}

#define DO_CV_IMSHOW true
#define DO_STREAM false

void imshow(std::string name, const cv::Mat& image){
    if(DO_STREAM){
        std::vector<uchar> buff;
        cv::imencode(".jpg", image, buff, params);
        streamer.publish("/"+name, std::string(buff.begin(), buff.end()));
    }
    if(DO_CV_IMSHOW){
        cv::imshow(name, image);
    }
}
}