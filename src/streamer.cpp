#include "streamer.hpp"
#include <nadjieb/mjpeg_streamer.hpp>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <queue>

namespace streamer{

void streamThreadLoop();
#define DO_CV_IMSHOW false
#define DO_STREAM true

std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
nadjieb::MJPEGStreamer streamer;
std::vector<std::thread> pool;
const int num_stream_threads = 3;

void initStreaming(){
    streamer.start(8080);
    if(DO_STREAM){
        for(int i = 0; i < num_stream_threads; i++){
            pool.push_back(std::thread(streamThreadLoop));
        }
    }
}

struct StreamData{
    cv::Mat image;
    std::string name;
};

std::mutex queue_lock;
std::queue<StreamData> queue;

void imshow(std::string name, const cv::Mat& image){
    if(DO_CV_IMSHOW){
        cv::imshow(name, image);
    }
    if(DO_STREAM){
        auto start_time = std::chrono::high_resolution_clock::now();
        queue_lock.lock();
        auto wait_duration = std::chrono::high_resolution_clock::now() - start_time;
        if(wait_duration > std::chrono::microseconds(100)){
            std::cout
                << name
                << " waited for stream thread for (us): "
                << std::chrono::duration_cast<std::chrono::microseconds>(wait_duration).count()
                << "\n";
        }
        queue.push(StreamData{image, name});
        queue_lock.unlock();
    }
}

bool running = true;

void streamThreadLoop(){
    while(running){
        if(!queue.empty()){
            queue_lock.lock();
            if(queue.empty()){
                // someone got to the data before us
                queue_lock.unlock();
                continue;
            }
            StreamData data = queue.front();
            queue.pop();
            queue_lock.unlock();

            int img_type = data.image.type();
            if(img_type == CV_32FC1 || img_type == CV_32FC3 || img_type == CV_64FC1 || img_type == CV_64FC3){
                data.image *= 255;
            }
            std::vector<uchar> buff;
            cv::imencode(".jpg", data.image, buff, params);
            streamer.publish("/"+data.name, std::string(buff.begin(), buff.end()));
        }
        // TODO: use condition_variable
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
}

void closeThread(){
    running = false;
    for(int i = 0; i < num_stream_threads; i++){
        pool[i].join();
    }
}
}