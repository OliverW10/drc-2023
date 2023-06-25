#include "arrow.hpp"
#include <Eigen/Core>
#include <vector>
#include "config.hpp"
#include "vision.hpp"
#include "streamer.hpp"
#include <string>
#include <opencv2/opencv.hpp>
#include <filesystem>

int img_idx = 0;
std::string pre_filename = std::filesystem::path(__FILE__).parent_path().parent_path().string() + std::string("/images/dataset/img");
std::string post_filename = ".png";

#define DO_CREATE_DATASET false

// https://stackoverflow.com/questions/71596568/opencv4-5-5-error-assertion-failed-empty-in-cvdnndnn4-v20211220net
cv::dnn::Net arrow_classifier = cv::dnn::readNetFromONNX("../arrow/model.onnx");;

const double max_side_length = 40;
const double min_height = 0.1 * pixels_per_meter;
const double min_width = 0.05 * pixels_per_meter;

bool coarse_classify(const cv::Rect& rect, const std::vector<cv::Point>& contour){
    // check contour size is roughly correct
    if(
        rect.width < min_width || rect.width > max_side_length
        || rect.height < min_height || rect.height > max_side_length
    ){
        return false;
    }

    // exclude too far away
    if(rect.y < 1 * pixels_per_meter){
        return false;
    }

    // exclude too big
    double area = cv::contourArea(contour);
    double max_area = (max_side_length * max_side_length);
    if(area / max_area > 0.5){
        return false;
    }
    return true;
}

void draw_rects(const std::vector<cv::Rect>& rects, const cv::Mat& img, const cv::Scalar& color, const std::string& text){
    for(auto rect : rects){
        cv::rectangle(img, rect, color);
        cv::putText(img, text, rect.tl(), cv::FONT_HERSHEY_COMPLEX, 1, color);
    }
}

const double classify_threshold = 0.3;
const double arrow_alpha = 0.1;

void find_arrow(const cv::Mat& hsv_ground, double& confidence_out){
    cv::Mat mask;
    cv::inRange(hsv_ground, getConfigHsvScalarLow("black"), getConfigHsvScalarHigh("black"), mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> lefts;
    std::vector<cv::Rect> negatives;
    std::vector<cv::Rect> rights;

    double current_confidence = 0;
    for(auto contour : contours){
        cv::Rect rect = cv::boundingRect(contour);

        if(coarse_classify(rect, contour) == false){
            continue;
        }

        int diff_w = max_side_length - rect.width;
        int diff_h = max_side_length - rect.height;
        // create a new square rect centered on existing bouding rect
        cv::Rect rect_square(rect.x-diff_w/2, rect.y-diff_h/2, rect.width+diff_w, rect.height+diff_h);


        cv::Mat arrow_img(mask, rect_square);

        cv::Mat arrow_blob = cv::dnn::blobFromImage(arrow_img);
        arrow_classifier.setInput(arrow_blob);
        auto x = arrow_classifier.getLayerNames();
        cv::Mat network_result = arrow_classifier.forward();
        double combined = network_result.at<float>(0) - network_result.at<float>(2);
        streamer::imshow("arrow", arrow_img);

        if(std::abs(combined) > std::abs(current_confidence)){
            current_confidence = combined;
        }

        if(combined > classify_threshold){
            lefts.push_back(rect_square);
        }else if(combined < -classify_threshold){
            rights.push_back(rect_square);
        }else{
            negatives.push_back(rect_square);
        }

        if(DO_CREATE_DATASET){
            cv::imwrite(pre_filename + std::to_string(img_idx) + post_filename, arrow_img);
        }
        img_idx ++;
    }

    confidence_out = arrow_alpha * current_confidence + (1-arrow_alpha) * confidence_out;

    cv::Mat map_annotated;
    cv::cvtColor(mask, map_annotated, cv::COLOR_GRAY2BGR);
    draw_rects(negatives, map_annotated, cv::Scalar(0, 0, 255), "");
    draw_rects(lefts, map_annotated, cv::Scalar(0, 255, 0), "left");
    draw_rects(rights, map_annotated, cv::Scalar(255, 0, 0), "right");

    streamer::imshow("black-mask", map_annotated);
}
