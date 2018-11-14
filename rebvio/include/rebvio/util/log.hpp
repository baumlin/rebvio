/*
 * log.hpp
 *
 *  Created on: Nov 12, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_UTIL_LOG_HPP_
#define INCLUDE_REBVIO_UTIL_LOG_HPP_

#include <memory>
#include <spdlog/spdlog.h>

namespace rebvio {
namespace util {

class Log {
public:
	static void init();
	inline static std::shared_ptr<spdlog::logger>& getConsole() { return console_logger_; }
	inline static std::shared_ptr<spdlog::logger>& getOdometry() { return odometry_logger_; }

private:
	static std::shared_ptr<spdlog::logger> console_logger_;
	static std::shared_ptr<spdlog::logger> odometry_logger_;
};


} /* namespace util */


#define REBVIO_TRACE(...) rebvio::util::Log::getConsole()->trace(__VA_ARGS__)
#define REBVIO_DEBUG(...) rebvio::util::Log::getConsole()->debug(__VA_ARGS__)
#define REBVIO_INFO(...) rebvio::util::Log::getConsole()->info(__VA_ARGS__)
#define REBVIO_WARN(...)  rebvio::util::Log::getConsole()->warn(__VA_ARGS__)
#define REBVIO_ERROR(...) rebvio::util::Log::getConsole()->error(__VA_ARGS__)
#define REBVIO_FATAL(...) rebvio::util::Log::getConsole()->fatal(__VA_ARGS__)

#ifdef TESTING
  #define REBVIO_ODOMETRY(...) rebvio::util::Log::getOdometry()->info(__VA_ARGS__)
#else
  #define REBVIO_ODOMETRY(...)
#endif

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_UTIL_LOG_HPP_ */
