#include "obstacle.hpp"
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "streamer.hpp"

const double v = 1/3;
const cv::Mat edge_kernel = (cv::Mat_<double>(3, 3) <<
    0, 0, 0,
    v, v, v,
    -v, -v, -v
);
const cv::Mat dilate_kernal = cv::Mat(cv::Size(11, 11), CV_8UC1, cv::Scalar(1));


// takes in all the buffers it needs to avoid allocation
void find_obsticles(
    const cv::Mat& hsv_ground,
    cv::Mat& purple_mask,
    cv::Mat& red_mask,
    cv::Mat& purple_obstacle,
    cv::Mat& red_obstacle,
    cv::Mat& output
){
    cv::inRange(hsv_ground, getConfigHsvScalarLow("purple"), getConfigHsvScalarHigh("purple"), purple_mask);
    cv::inRange(hsv_ground, getConfigHsvScalarLow("red"), getConfigHsvScalarHigh("red"), red_mask);
    red_mask = 255-red_mask;
    streamer::imshow("red", red_mask);
    streamer::imshow("purple", purple_mask);

    cv::filter2D(red_mask, red_obstacle,       -1, edge_kernel);
    cv::threshold(red_obstacle, red_obstacle,      200, 255*1, cv::THRESH_BINARY);

    cv::filter2D(purple_mask, purple_obstacle, -1, edge_kernel);
    cv::threshold(purple_obstacle, purple_obstacle, 200, 255*0.7, cv::THRESH_BINARY);

    red_obstacle += purple_obstacle;
    cv::dilate(red_obstacle, red_obstacle, dilate_kernal);

    red_obstacle.convertTo(output, CV_32FC1, 255);
    streamer::imshow("obstacle", output);
}