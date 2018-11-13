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
	inline static std::shared_ptr<spdlog::logger>& get() { return logger_; }

private:
	static std::shared_ptr<spdlog::logger> logger_;
};


} /* namespace util */


#define REBVIO_TRACE(...) rebvio::util::Log::get()->trace(__VA_ARGS__)
#define REBVIO_DEBUG(...) rebvio::util::Log::get()->debug(__VA_ARGS__)
#define REBVIO_INFO(...) rebvio::util::Log::get()->info(__VA_ARGS__)
#define REBVIO_WARN(...)  rebvio::util::Log::get()->warn(__VA_ARGS__)
#define REBVIO_ERROR(...) rebvio::util::Log::get()->error(__VA_ARGS__)
#define REBVIO_FATAL(...) rebvio::util::Log::get()->fatal(__VA_ARGS__)

} /* namespace rebvio */

#endif /* INCLUDE_REBVIO_UTIL_LOG_HPP_ */
