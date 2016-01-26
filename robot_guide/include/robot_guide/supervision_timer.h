/**
  supervision_timer.h
  Author: Michelangelo Fiore
 
  this class represents a timer, which can be set to a specific duration, and can be stopped.
 */


#ifndef SUPERVISION_TIMER_H
#define SUPERVISION_TIMER_H

#include <ros/ros.h>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

class SupervisionTimer {
public:
	SupervisionTimer(double seconds_to_wait);
	//stops the current timer
	void stop();
	//starts the timer, the function is blocking and so it should run in its thread if we want to be able to stop it.
	void start();
	//true when the timer has elapsed.
	bool isElapsed();
	bool isRunning();
private:
	double seconds_to_wait_;
	bool is_elapsed_;
	bool is_running_;
	bool stop_;
	boost::mutex mutex_is_elapsed_;
	boost::mutex mutex_stop_;
	boost::mutex mutex_is_running_;

};

#endif