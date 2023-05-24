#include "streamer.hpp"
#include <nadjieb/mjpeg_streamer.hpp>
#include <thread>
#include <iostream>
#include <atomic>
namespace streamer{

static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
static nadjieb::MJPEGStreamer streamer;

static std::atomic_int alive_thread_count = 0;

void initStreaming(){
    streamer.start(8080);
}

#define DO_CV_IMSHOW true
#define DO_STREAM true


void stream_image(const std::string& name, const cv::Mat* image, bool copy){
    if(image->empty()){
        std::cout << "got empty image thread\n";
        return;
    }
    cv::Mat img_copy;
    if(copy){
        image->copyTo(img_copy);
    }else{
        img_copy = *image;
    }
    alive_thread_count --;

    std::vector<uchar> buff;
    cv::imencode(".jpg", img_copy, buff, params);
    streamer.publish("/"+name, std::string(buff.begin(), buff.end()));
    // if we didnt copied the image it means it was copied in imshow and is on the heap
    // since it was a copy when we recived it the const wasn't meant to be there anyway
    if(!copy){
        free((cv::Mat*)image);
    }
}

void imshow(const std::string& name, const cv::Mat& image, bool copy_immediately){
    if(DO_CV_IMSHOW){
        cv::imshow(name, image);
    }
    if(DO_STREAM){
        if(image.empty()){
            std::cout << "got empty image imshow\n";
            return;
        }
        // If copy_immediately is true create a copy of the image on the heap on the main thread
        // before returning to avoid it being deallocated before it would be copied on the new thread
        // otherwise give the new thread a ptr to the existing image which it will copy
        const cv::Mat* img_copy;
        if(copy_immediately){
            img_copy = new cv::Mat(image);
            image.copyTo(*img_copy);
        }else{
            img_copy = &image;
        }
        alive_thread_count ++;
        std::thread t(stream_image, std::cref(name), img_copy, !copy_immediately);
        t.detach();
    }
}

void wait_for_threads(){
    int x = 0;
    while(alive_thread_count > 0){
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        x++;
        if(x > 10000){
            std::cout << "Waited too long for streamer threads\n";
            break;
        }
    }
}
}