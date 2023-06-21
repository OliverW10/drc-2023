#include "obstacle.hpp"
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "streamer.hpp"

const double v = 1/3;
const cv::Mat edge_kernel = (cv::Mat_<double>(3, 3) <<
    0, 0, 0,
    1, 1, 1,
    -1, -1, -1
) / 3;
// const cv::Mat edge_kernel = (cv::Mat_<double>(3, 3) <<
//     v, v, v,
//     v, v, v,
//     v, v, v
// );
const cv::Mat edge_kernel_x = (cv::Mat_<double>(1, 3) << 1, 1, 1);
const cv::Mat edge_kernel_y = (cv::Mat_<double>(3, 1) << 0, v, -v);

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
    // Red - Other cars
    // cv::inRange(hsv_ground, getConfigHsvScalarLow("red"), getConfigHsvScalarHigh("red"), red_mask);
    // red_mask = 255-red_mask;
    // streamer::imshow("red", red_mask);
    // if(use_seperable){
    //     cv::sepFilter2D(red_mask, red_obstacle, -1, edge_kernel_x, edge_kernel_y);
    // }else{
    //     cv::filter2D(red_mask, red_obstacle,       -1, edge_kernel);
    // }
    // cv::threshold(red_obstacle, red_obstacle,       200, 255*0.3, cv::THRESH_BINARY);

    // Purple - Boxes
    cv::inRange(hsv_ground, getConfigHsvScalarLow("purple"), getConfigHsvScalarHigh("purple"), purple_mask);
    streamer::imshow("purple", purple_mask);
    #if 0
        // is slower for some reason
        // TODO: test on pi
        cv::sepFilter2D(purple_mask, purple_obstacle, -1, edge_kernel_x, edge_kernel_y);
    #else
        cv::filter2D(purple_mask, purple_obstacle, -1, edge_kernel);
    #endif
    cv::threshold(purple_obstacle, purple_obstacle, 200, 255*0.7, cv::THRESH_BINARY);

    // Combine both obstacles maps to do a single dilate and convert
    // red_obstacle += purple_obstacle;
    cv::dilate(purple_obstacle, purple_obstacle, dilate_kernal);
    purple_obstacle.convertTo(output, CV_32FC1, 1.0/255);

    streamer::imshow("obstacle", output);
}