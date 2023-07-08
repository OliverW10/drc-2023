#include <opencv2/opencv.hpp>
#include "finish_line.hpp"
#include "config.hpp"
#include "streamer.hpp"

cv::Rect get_finish_line_roi(cv::Size size){
    double rect_y = getConfigDouble("finish_rect_y");
    cv::Rect_<double> rect(0, rect_y, 1, 1-rect_y);
    cv::Rect rect_pixels(
        rect.x * size.width,
        rect.y * size.height,
        rect.width * size.width,
        rect.height * size.height
    );
    return rect_pixels;
}

double streak = 0;
cv::Mat hsv;
cv::Mat mask;

void find_finish_line(const cv::Mat& image, double& _finish_line_conf){
    double required_repetitions = (int)getConfigDouble("finish_required_streak");
    double required_area = getConfigDouble("finish_required_area");

    cv::Rect rect_pixels = get_finish_line_roi(image.size());
    cv::Mat region(image, rect_pixels);
    cv::cvtColor(region, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, getConfigHsvScalarLow("green"), getConfigHsvScalarHigh("green"), mask);
    int pixel_count = cv::sum(mask).val[0];
    bool current_has_finish_line = pixel_count > required_area * rect_pixels.area();
    if(current_has_finish_line){
        streak ++;
    }else{
        streak = 0;
    }
    _finish_line_conf = streak / required_repetitions;

    streamer::imshow("green", mask);
}
