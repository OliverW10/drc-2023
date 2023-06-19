#ifndef CONFIG_H
#define CONFIG_H

#include <opencv2/opencv.hpp>
#include <string>

double getConfigDouble(std::string key);
std::string getConfigString(std::string key);

void tryUpdateConfig();
void tryUpdateConfig(std::string filename);

cv::Scalar getConfigHsvScalarLow(std::string name);
cv::Scalar getConfigHsvScalarHigh(std::string name);


#endif // CONFIG_H