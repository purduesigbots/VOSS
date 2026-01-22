#pragma once 

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/chassis/HoloChassis.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "api.h"
#include <vector>
#include <any>

namespace ssov{

class Logger{
    public:

        enum class log_type { INT, FLOAT, DOUBLE, BOOL, CHASSIS, ODOM, None };

        struct log_item {
            const char* name = "Unknown Variable";
            std::any ptr;
        };

        struct log_item_datatype {
            const char* name = "Unknown Variable";
            std::any ptr;
            log_type type;
        };

        Logger(std::vector<log_item>& items_, int delay);
        ~Logger();
        
        void start_log();

        void toggle_log();

        void stop_log();

    private:
        std::vector<log_item> items;
        std::vector<log_item_datatype> items_type;
        pros::Task* task = nullptr;
        int time_delay = 10;

        static void task_entry(void* param);
        void run();


};

}



