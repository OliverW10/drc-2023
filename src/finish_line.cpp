#include <opencv2/opencv.hpp>
#include "finish_line.hpp"
#include "config.hpp"
#include "streamer.hpp"

const cv::Rect_<double> region(0, 0.7, 1, 0.3);
const double required_area = 0.1;
const double alpha = 0.05;

double finish_line_confidence = 0;

void find_finish_line(const cv::Mat& image, double& _finish_line_conf){
    cv::Rect rect_pixels(
        region.x * image.cols,
        region.y * image.rows,
        region.width * image.cols,
        region.height * image.rows
    );
    cv::Mat region(image, rect_pixels);
    cv::Mat hsv;
    cv::cvtColor(region, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, getConfigHsvScalarLow("green"), getConfigHsvScalarHigh("green"), mask);
    int count = cv::sum(mask).val[0];
    bool current_has_finish_line = count > required_area * rect_pixels.area();
    finish_line_confidence = current_has_finish_line * alpha + finish_line_confidence * (1-alpha);
    _finish_line_conf = finish_line_confidence;

    streamer::imshow("green", mask);
}
