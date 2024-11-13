#pragma once

#include <fstream>

#include "json.hpp"
#include "pros/rtos.hpp"

namespace replay {
    class Packet {
        private:
            nlohmann::json data;
        public:
            Packet();
            void add_variable(std::string name, double value);
            void add_point(std::string name, double x, double y);
            void add_pose(std::string name, double x, double y, double theta);
            std::string to_string();
    };
    class FileLogger {
        private:
            std::ofstream file;
            pros::Mutex mutex;
        public:
            FileLogger(std::string filename);
            void log(Packet packet);
            void close();
    };
}