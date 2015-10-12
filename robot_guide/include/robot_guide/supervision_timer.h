#ifndef SUPERVISION_TIMER_H
#define SUPERVISION_TIMER_H

#include <ros/ros.h>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

class SupervisionTimer {
public:
	SupervisionTimer(double seconds_to_wait);
	void stop();
	void start();
	bool isElapsed();
private:
	double seconds_to_wait_;
	bool is_elapsed_;
	bool stop_;
	boost::mutex mutex_is_elapsed_;
	boost::mutex mutex_stop_;

};

#endif