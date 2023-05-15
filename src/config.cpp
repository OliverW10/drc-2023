#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <stdio.h>
#include "config.hpp"
#include <iostream>


static long file_modified_time = 0;
static std::unordered_map<std::string, double> config;


double getConfig(std::string key){
    if (config.find(key) == config.end()){
        std::cout << "tried to access config: " << key << " which dosen't exist";
        return 0;
    }
    return config.at(key);
}

std::unordered_map<std::string, double> readConfig(std::string filename){
    std::ifstream infile(filename);
    std::unordered_map<std::string, double> map;

    std::string line;
    std::getline(infile, line); // consume first line
    printf("reading config file: %s\n", filename.c_str());
    while (std::getline(infile, line))
    {
        int comma_idx = line.find(',');
        std::string key = line.substr(0, comma_idx);
        int left = line.length() - comma_idx;
        std::string value_str = line.substr(comma_idx+1, left);
        // std::cout << "key: " << key << ", val: " << value_str << std::endl;
        double value = std::stod(value_str);
        map.emplace(key, value);
        // printf("%s -> %f\n", line.substr(0, comma_idx-1).c_str(), std::stod(line.substr(comma_idx).c_str()));
    }
    return map;
}

void tryUpdateConfig(std::string filename){
    long new_modified_time = std::filesystem::last_write_time(filename).time_since_epoch().count();;
    if(new_modified_time != file_modified_time){
        config = readConfig(filename);
    }
    file_modified_time = new_modified_time;
}

// overload uses default config path
void tryUpdateConfig(){
    std::filesystem::path project_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
    std::filesystem::path default_config_path = project_dir.append("config.csv");
    tryUpdateConfig(default_config_path);
}

