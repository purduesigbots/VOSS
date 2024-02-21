#include <iostream>
#include <sstream>

class Logger {
  private:
    enum Level { INFO = 1, WARN = 2, ERROR = 4, DEBUG = 8 };

    static int level;

  public:
    Logger() = delete;

    static void set_level(int level) {
        Logger::level = level;
    }

    template <typename T> static void info(std::string message, T value = "") {
        if (!(level & INFO))
            return;

        std::stringstream ss;
        ss << message << value << std::endl;
        std::cout << ss.str();
    }

    template <typename T> static void warn(std::string message, T value = "") {
        if (!(level & WARN))
            return;

        std::stringstream ss;
        ss << message << value << std::endl;
        std::cerr << ss.str();
    }

    template <typename T> static void error(std::string message, T value = "") {
        if (!(level & ERROR))
            return;

        std::stringstream ss;
        ss << message << value << std::endl;
        std::cerr << ss.str();
    }

    template <typename T> static void debug(std::string message, T value = "") {
        if (!(level & DEBUG))
            return;

        std::stringstream ss;
        ss << message << value << std::endl;
        std::clog << ss.str();
    }
};