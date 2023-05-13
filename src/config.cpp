#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <stdio.h>
#include "config.hpp"

// using namespace ;

static long file_modified_time = 0;
static std::unordered_map<std::string, double> config;


double getConfig(std::string key){
    return config.at(key);
}

void tryUpdateConfig(std::string filename){
    long new_modified_time = std::filesystem::last_write_time(std::filesystem::path("config.csv")).time_since_epoch().count();;
    if(new_modified_time != file_modified_time){
        config = readConfig(filename);
    }
    file_modified_time = new_modified_time;
}

void tryUpdateConfig(){
    std::filesystem::path project_dir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path default_config_path = project_dir.append("config.csv");
    tryUpdateConfig(default_config_path);
}

std::unordered_map<std::string, double> readConfig(std::string filename){
    std::ifstream infile(filename);
    std::unordered_map<std::string, double> map;

    std::string line;
    std::getline(infile, line); // consume first line
    printf("reading config file: %s\n", filename);
    while (std::getline(infile, line))
    {
        int comma_idx = line.find(',');
        map.emplace(line.substr(0, comma_idx-1), std::stod(line.substr(comma_idx).c_str()));
        printf("%s -> %f\n", line.substr(0, comma_idx-1), std::stod(line.substr(comma_idx).c_str()));
    }
    return map;
}
