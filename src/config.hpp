#ifndef CONFIG_H
#define CONFIG_H

#include <string>

double getConfigDouble(std::string key);
std::string getConfigString(std::string key);

void tryUpdateConfig();
void tryUpdateConfig(std::string filename);

#endif // CONFIG_H