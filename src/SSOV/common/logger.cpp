#include "SSOV/common/logger.hpp"

namespace ssov{

Logger::Logger(std::vector<log_item>& items_, int time_delay):
        items(items_),
        time_delay(time_delay){};

Logger::~Logger(){
    stop_log();
}

void Logger::start_log(){
     for (auto& item : Logger::items){
            items_type.push_back({item.name, item.ptr, log_type::None});
            auto& typed_item = items_type.back();
        const std::type_info& type = item.ptr.type();
            if (type == typeid(ssov::DiffChassis*)){
                typed_item.type = Logger::log_type::CHASSIS;
            }
            else if (type == typeid(ssov::HolonomicChassis*)){
                typed_item.type = Logger::log_type::CHASSIS;
            }
            else if (type == typeid(ssov::Localizer*)){
                typed_item.type = Logger::log_type::ODOM;
            }
            else if (type == typeid(int*)){
                typed_item.type = Logger::log_type::INT;
            }
            else if (type == typeid(float*)){
                typed_item.type = Logger::log_type::DOUBLE;
            }
            else if (type == typeid(double*)){
                typed_item.type = Logger::log_type::DOUBLE;
            }
            else if (type == typeid(bool*)){
                typed_item.type = Logger::log_type::BOOL;
            }
    }
    task = new pros::Task(Logger::task_entry, this, "LoggerTask");
}

void Logger::stop_log(){
    if (task){
        task->remove();
        delete task;
        task = nullptr;
    }
    else{
        printf("Logger task has not been created");
    }
}

void Logger::toggle_log(){
    if (task){
        task->notify();
        printf("Logger Toggled");
    }
    else{
        printf("Logger task has not been created");
    }
}

void Logger::task_entry(void* param){
    static_cast<Logger*>(param)->run();
}

void Logger::run(){
    bool running = true;
    while(true){
        if (running){
            for(auto& item : items_type){
                std::cout << item.name << ": ";
                switch (item.type){
                    case Logger::log_type::CHASSIS:
                        auto* chassis = std::any_cast<ssov::DiffChassis*>(item.ptr);
                        auto signal = chassis->get_current_drive_signal();
                        std::cout << "X power: " << signal.x << ", Y power: " << signal.y << ", Theta power: " << signal.theta << std::endl;
                        break;
                    case Logger::log_type::ODOM:
                        auto* odom = std::any_cast<ssov::Localizer*>(item.ptr);
                        auto pose = odom->get_pose();
                        std::cout << "X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
                        break;
                    case Logger::log_type::INT:
                        int* value_int = std::any_cast<int*>(item.ptr);
                        std::cout << value_int << std::endl;
                        break;
                    case Logger::log_type::FLOAT:
                        float* value_float = std::any_cast<float*>(item.ptr);
                        std::cout << value_float << std::endl;
                    case Logger::log_type::DOUBLE:
                        double* value_double = std::any_cast<double*>(item.ptr);
                        std::cout << value_double << std::endl;
                    case Logger::log_type::BOOL:
                        bool* value_bool = std::any_cast<bool*>(item.ptr);
                        std::cout << value_bool << std::endl;
                    default:
                        std::cout << "Value with unknown datatype" << std::endl;
                        break;
                }

            }
        }
        if (pros::Task::notify_take(true,0) > 0){
            running = !running;
        }
        pros::delay(time_delay);
    }
}

}