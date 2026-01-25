#include "SSOV/common/logger.hpp"

namespace ssov{

Logger::Logger(std::vector<log_item>& items_, int time_delay):
        items(items_),
        time_delay(time_delay){};

Logger::~Logger(){
    stop_log();
}

void Logger::start_log(){

    printf("Initalizing Logger\n");
    printf("Logger Items Length: %d\n", Logger::items.size());
    // Copying data from the input data (variable and name)
     for (auto& item : Logger::items){
            items_type.push_back({item.name, item.ptr, log_type::None});
            auto& typed_item = items_type.back();
    //Checking the data type of the pointer and assigning one of the enums: CHASSIS, ODOM, INT, DOUBLE, BOOL
        const std::type_info& type = item.ptr.type();
            if (type == typeid(std::shared_ptr<ssov::DiffChassis>)){
                typed_item.type = Logger::log_type::CHASSIS;
            }
            else if (type == typeid(std::shared_ptr<ssov::HolonomicChassis>)){
                typed_item.type = Logger::log_type::CHASSIS;
            }
            else if (type == typeid(std::shared_ptr<ssov::TrackingWheelLocalizer>)){
                typed_item.type = Logger::log_type::ODOM;
            }
            else if (type == typeid(int*)){
                typed_item.type = Logger::log_type::INT;
            }
            else if (type == typeid(float*)){
                typed_item.type = Logger::log_type::FLOAT;
            }
            else if (type == typeid(double*)){
                typed_item.type = Logger::log_type::DOUBLE;
            }
            else if (type == typeid(bool*)){
                typed_item.type = Logger::log_type::BOOL;
            }
    }

    for (auto item : items_type){
        std::cout << static_cast<int>(item.type) << std::endl;
    }
    
    // Creating the logging task
    printf("Creating task\n");
    this->task = new pros::Task(Logger::task_entry, this);
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

// Function that allows us to pause and resume the logger without stopping it or the program
void Logger::toggle_log(){
    if (task){
        task->notify();
        printf("Logger Toggled");
    }
    else{
        printf("Logger task has not been created");
    }
}

// Handles making the parameters to the task static
void Logger::task_entry(void* param){
    printf("Task Entry\n");
    static_cast<Logger*>(param)->run();
}

//The code that is being run inside the task
void Logger::run(){
    bool running = true;
    printf("Logger Started\n");
    while(true){
        // Running is the variable that is toggled to pause/resume the task
        if (running){
        // For each item input to log it prints the name, checks the data type enum value, then prints according to the data type enum
            for(auto item : items_type){
                //std::cout << "Brain ache" << std::endl;
                std::cout << item.name << ": ";
                switch (item.type){
                    case Logger::log_type::CHASSIS: {
                        auto chassis_log = std::any_cast<std::shared_ptr<ssov::HolonomicChassis>>(item.ptr);
                        auto signal_log = chassis_log->get_current_drive_signal();
                        std::cout << "X power: " << signal_log.x << ", Y power: " << signal_log.y << ", Theta power: " << signal_log.theta << std::endl;
                        break;
                    }
                    case Logger::log_type::ODOM: {
                        auto odom_log = std::any_cast<std::shared_ptr<ssov::TrackingWheelLocalizer>>(item.ptr);
                        auto pose_log = odom_log->get_pose();
                        std::cout << "X: " << pose_log.x << ", Y: " << pose_log.y << ", Theta: " << pose_log.theta << std::endl;
                        break;
                    }
                    case Logger::log_type::INT: {
                        int* value_int_log = std::any_cast<int*>(item.ptr);
                        std::cout << *value_int_log << std::endl;
                        break;
                    }
                    case Logger::log_type::FLOAT: {
                        float* value_float_log = std::any_cast<float*>(item.ptr);
                        std::cout << *value_float_log << std::endl;
                    }
                    case Logger::log_type::DOUBLE: {
                        double* value_double_log = std::any_cast<double*>(item.ptr);
                        std::cout << *value_double_log << std::endl;
                    }
                    case Logger::log_type::BOOL: {
                        bool* value_bool_log = std::any_cast<bool*>(item.ptr);
                        std::cout << *value_bool_log << std::endl;
                    }
                    default:
                        std::cout << "Value with unknown datatype" << std::endl;
                        break;
                }

            }
        }
        // Checks to see if the task recieved a notificaion and flipps the running boolian if it got one. Clears the notificaion after grabbing it.
        if (pros::Task::notify_take(true,0) > 0){
            running = !running;
        }
        pros::delay(time_delay);
    }
}

}