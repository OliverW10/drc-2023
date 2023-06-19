#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <stdio.h>
#include "config.hpp"
#include <iostream>


static long file_modified_time = 0;
static std::unordered_map<std::string, std::string> config;

bool configExists(std::string key){
    if (config.find(key) == config.end()){
        std::cout << "tried to access config: " << key << " which dosen't exist";
        return false;
    }
    return true;
}

double getConfigDouble(std::string key){
    if(configExists(key)){
        return std::stod(config.at(key));
    }else{
        return 0;
    }
}

std::string getConfigString(std::string key){
    if (configExists(key)){
        return config.at(key);
    }else{
        return "";
    }
}

std::unordered_map<std::string, std::string> readConfig(std::string filename){
    std::ifstream infile(filename);
    std::unordered_map<std::string, std::string> map;

    std::string line;
    std::getline(infile, line); // consume first line
    printf("reading config file: %s\n", filename.c_str());
    while (std::getline(infile, line))
    {
        int comma_idx = line.find(',');
        std::string key = line.substr(0, comma_idx);
        int left = line.length() - comma_idx;
        std::string value = line.substr(comma_idx+1, left);
        // std::cout << "key: " << key << ", val: " << value_str << std::endl;
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


cv::Scalar getConfigHsvScalarLow(std::string name){
    return cv::Scalar(
        getConfigDouble(name+"_h_low"), 
        getConfigDouble(name+"_s_low"), 
        getConfigDouble(name+"_v_low")
    );
}

cv::Scalar getConfigHsvScalarHigh(std::string name){
    return cv::Scalar(
        getConfigDouble(name+"_h_high"), 
        getConfigDouble(name+"_s_high"), 
        getConfigDouble(name+"_v_high")
    );
}