#pragma once 

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/chassis/HoloChassis.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/localizer/TrackingWheelLocalizer.hpp"
#include "api.h"
#include <vector>
#include <any>

namespace ssov{

class Logger{
    public:
        // Struct to represent each value that is being logged with a name and pointer. It is the struct the user is dealing with
        struct log_item {
            std::any ptr;
            const char* name = "Unknown Variable";
        };

        // Creates the logger object, the user passes a vector containing the pointer to the variable and then a string of the name they want to print with it
        Logger(std::vector<log_item>& items_, int delay);
        ~Logger();
        
        // Starts the logger
        void start_log();
        
        // Toggles the logger without suspending or killing the task
        void toggle_log();

        // Kills the task
        void stop_log();

        pros::Task* task = nullptr;

    private:
        // Enum created to represent each of the data types the logger is able to handle
        enum class log_type { INT, FLOAT, DOUBLE, BOOL, CHASSIS, ODOM, None };

        // Background struct that represents each object being logged but also includes the data type of the object for handling the correct printing
        struct log_item_datatype {
            const char* name = "Unknown Variable";
            std::any ptr;
            log_type type;
        };
        
        // Two vectors: items is the input to the logger that the user gives, items_type is used in the background once the type of the object is determined 
        std::vector<log_item> items;
        std::vector<log_item_datatype> items_type;

        // Variable to determine the delay in the logger. Defaults at 10ms
        int time_delay = 10;

        static void task_entry(void* param);
        void run();


};

}



