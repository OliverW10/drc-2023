#include "vision.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include "config.hpp"
#include <string>

static int img_idx = 0;
static std::string pre_filename = std::string(__FILE__) + std::string("/../images/dataset/img");
static std::string post_filename = ".png";

#define DO_CREATE_DATASET true

void find_potential_arrow_contours(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& output_contours){
    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(mask, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(auto contour : all_contours){
        cv::Rect rect = cv::boundingRect(contour);

        double max_side_length = 0.4 * pixels_per_meter;
        double min_height = 0.1 * pixels_per_meter;
        double min_width = 0.05 * pixels_per_meter;
        // check contour size is roughly correct
        if(
            rect.width < min_width
            || rect.width > max_side_length
            || rect.height < min_height
            || rect.height > max_side_length
        ){continue;};
        
        // exclude too far away
        if(rect.y < 1 * pixels_per_meter) continue;

        // exclude too big
        double area = cv::contourArea(contour);
        double max_area = (max_side_length * max_side_length);
        if(area / max_area > 0.5) continue;

        if(DO_CREATE_DATASET){
            int diff_w = max_side_length - rect.width;
            int diff_h = max_side_length - rect.height;
            // create a new square rect centered on existing bouding rect
            cv::Rect new_rect(rect.x-diff_w/2, rect.y-diff_h/2, rect.width+diff_w, rect.height+diff_h);

            cv::Mat arrow_img(mask, new_rect);
            cv::imwrite(pre_filename + std::to_string(img_idx) + post_filename, arrow_img);
            img_idx ++;
        }
        output_contours.push_back(contour);
    }
}

double find_arrow(const cv::Mat& hsv_ground){
    cv::Mat mask_black;
    cv::inRange(hsv_ground, getConfigHsvScalarLow("black"), getConfigHsvScalarHigh("black"), mask_black);
    std::vector<std::vector<cv::Point>> contours{};
    find_potential_arrow_contours(mask_black, contours);

    cv::Mat map_annotated = cv::Mat::zeros(cv::Size(map_width_p, map_height_p), CV_8UC3);
    if(contours.size()){
        cv::drawContours(map_annotated, contours, -1, cv::Scalar(0, 0, 255));
    }

    // streamer::imshow("black-mask", mask_black);
    // streamer::imshow("black-good", map_annotated);

    return 0;
}