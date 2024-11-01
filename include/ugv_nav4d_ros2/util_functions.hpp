#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>

std::string generateTimestampedFilename(const std::string& suffix) {
    // Get current time as a time_t object
    auto now = std::chrono::system_clock::now();
    std::time_t nowTimeT = std::chrono::system_clock::to_time_t(now);

    // Convert to local time and format it
    std::tm nowTm;
    localtime_r(&nowTimeT, &nowTm); // Use localtime_r for Unix systems
    // Format the timestamp as "YYYYMMDD_HHMMSS"
    std::ostringstream oss;
    oss << std::put_time(&nowTm, "%Y%m%d_%H%M%S");

    // Construct the final filename with suffix
    return "ugv_nav4d_ros2_mls_" + oss.str() + suffix; // e.g., "_20231101_120505.bin"
}
