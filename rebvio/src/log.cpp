/*
 * log.cpp
 *
 *  Created on: Nov 12, 2018
 *      Author: baumlin
 */

#include "rebvio/util/log.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <ctime>

namespace rebvio {
namespace util {

std::shared_ptr<spdlog::logger> Log::console_logger_;
std::shared_ptr<spdlog::logger> Log::odometry_logger_;

void Log::init() {
	// Set the format for the console logger
	spdlog::set_pattern("%^[%T.%f] [%n] [%l]: %v%$");
	console_logger_ = spdlog::stdout_color_mt("Rebvio");
	console_logger_->set_level(spdlog::level::trace);

#ifdef TESTING
	// in case the odometry file output is enabled, get the date and time for the file name and set up the file logger
	try {
		time_t raw_time;
		struct tm* time_info;
		char buf[80];
		time(&raw_time);
		time_info = localtime(&raw_time);
		strftime(buf,sizeof(buf),"%Y-%m-%d_%H-%M-%S",time_info);
		odometry_logger_ = spdlog::basic_logger_st("Odometry",std::string(buf)+"_rebvio_odometry.txt");
		odometry_logger_->set_pattern("%v");
		odometry_logger_->set_level(spdlog::level::info);
	} catch (const spdlog::spdlog_ex& ex) {
		REBVIO_ERROR("Log::init failed: {}",ex.what());
	}
#endif
}

}
}


