#include <opencv2/core/mat.hpp>

void find_finish_line(const cv::Mat& image, double& has_finish_line);

cv::Rect get_finish_line_roi(cv::Size size);
