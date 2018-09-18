/*
 * timer.hpp
 *
 *  Created on: Sep 1, 2018
 *      Author: baumlin
 */

#ifndef INCLUDE_REBVIO_UTIL_TIMER_HPP_
#define INCLUDE_REBVIO_UTIL_TIMER_HPP_

#include <iostream>
#include <chrono>
#include <string>

namespace rebvio {
namespace util {

#ifdef TIMER
	#define REBVIO_TIMER_TICK()\
		static rebvio::util::Timer timer(__PRETTY_FUNCTION__); \
		timer.tick();
	#define REBVIO_NAMED_TIMER_TICK(NAME)\
		static rebvio::util::Timer timer_##NAME(#NAME);\
		timer_##NAME.tick();
	#define REBVIO_TIMER_TOCK() timer.tock();
	#define REBVIO_NAMED_TIMER_TOCK(NAME) timer_##NAME.tock();
#else
#define REBVIO_TIMER_TICK()
#define REBVIO_TIMER_TOCK()
#define REBVIO_NAMED_TIMER_TICK(NAME)
#define REBVIO_NAMED_TIMER_TOCK(NAME)
#endif

/*! \class Timer
 *  \brief Timer class to profile time spent in code sections.
 *
 *  To profile the time spent in a code section, set the preprocessor flag -DTIMER=ON and use the following two macros at the beginning and end of the section:
 *  \def REBVIO_TIMER_TICK(): Timer is named after the function within its scope it is called.
 *  \def REBVIO_NAMED_TIMER_TICK(NAME): Timer is named after input argument, e.g. REBVIO_NAMED_TIMER_TICK(myFunctionName).
 *  \def REBVIO_TIMER_TOCK():
 *  \def REBVIO_NAMED_TIMER_TOCK(NAME):
 *  This will instantiate a static Timer and print the profiling information upon its destruction.
 *
 */
class Timer {
public:
	Timer(std::string _name) : counter_(0), name_(_name), total_time_ms_(0) {}
	~Timer() { print();	}

	void tick() {
		tick_ = std::chrono::high_resolution_clock::now();
	}

	void tock () {
		++counter_;
		total_time_ms_ += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-tick_).count();
	}

private:
	void print() {
		std::cout<<"\n============== Timing Information ===============================\n";
		std::cout<<"Section: "<<name_<<"\n";
		std::cout<<"Counts: "<<counter_<<"\n";
		std::cout<<"Total Time: "<<total_time_ms_<<" [ms]\n";
		std::cout<<"Average Time: "<<total_time_ms_/double(counter_)<<" [ms]\n";
		std::cout<<"==================================================================\n";
	}

private:
	long int counter_;  //!< Number of passes
	std::string name_;	//!< Name of the timer
	std::chrono::high_resolution_clock::time_point tick_;  //!< Last tick
	double total_time_ms_;  //!< Total time spent in profiled section in [ms]
};


}
}


#endif /* INCLUDE_REBVIO_UTIL_TIMER_HPP_ */
