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

/*! \class Log
 *  \brief Logger class to write logging messages to the console or to file.
 *
 *  There are different severity levels (trace, debug, info, warn, error, and fatal). To write a log message to the console,
 *  use the logger macros according to the desired severity level, e.g.:
 *
 *  REBVIO_INFO("This is a message with int i={} and float f={:.{}f}",0,3.14,1);
 *
 *  This will print the message: This is a message with int i=0 and float f=3.1
 *  Note that the float precision can be set using the :.{}f syntax and the desired precision (1 in the example) as argument.
 *
 *  To print the odometry output (in camera coordinate frame) to a file (yyyy-mm-dd_hh-mm-ss_rebvio_odometry.txt), set the
 *  preprocessor flag -DTESTING=ON. The file is saved to the current working directory.
 */

class Log {
public:
	/*
	 * Initialize the logger properties (formats, output file, etc.).
	 */
	static void init();
	/*
	 * Return the Console Logger
	 */
	inline static std::shared_ptr<spdlog::logger>& getConsole() { return console_logger_; }

	/*
	 * Return the Odometry File Logger
	 */
	inline static std::shared_ptr<spdlog::logger>& getOdometry() { return odometry_logger_; }

private:
	static std::shared_ptr<spdlog::logger> console_logger_;  //!< Console Logger
	static std::shared_ptr<spdlog::logger> odometry_logger_; //!< Odometry File Logger
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
