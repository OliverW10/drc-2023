#ifndef CONFIG_H
#define CONFIG_H

#include <string>

double getConfig(std::string key);

void tryUpdateConfig();
void tryUpdateConfig(std::string filename);

#endif // CONFIG_H