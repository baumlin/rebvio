/*
 * log.cpp
 *
 *  Created on: Nov 12, 2018
 *      Author: baumlin
 */

#include "rebvio/util/log.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace rebvio {
namespace util {

std::shared_ptr<spdlog::logger> Log::logger_;

void Log::init() {
	spdlog::set_pattern("%^[%T.%f] [%n] [%l]: %v%$");
	logger_ = spdlog::stdout_color_mt("Rebvio");
	logger_->set_level(spdlog::level::trace);
}

}
}


