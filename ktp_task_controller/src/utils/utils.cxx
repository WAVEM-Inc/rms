#include "utils/utils.hxx"

std::string get_current_time()
{
    const std::chrono::_V2::system_clock::time_point &now = std::chrono::system_clock::now();
    const std::time_t &time = std::chrono::system_clock::to_time_t(now);
    const std::tm &tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%y%m%d%H%M%S") << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    return oss.str();
}