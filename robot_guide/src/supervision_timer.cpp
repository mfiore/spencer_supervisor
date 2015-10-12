#include <robot_guide/supervision_timer.h>

SupervisionTimer::SupervisionTimer(double seconds_to_wait) {
	seconds_to_wait_=seconds_to_wait;
	stop_=false;
	is_elapsed_=false;
}
void SupervisionTimer::stop() {
	boost::lock_guard<boost::mutex> lock(mutex_stop_);
	stop_=true;
}
void SupervisionTimer::start() {
	mutex_stop_.lock();
	stop_=false;
	mutex_stop_.unlock();
	mutex_is_elapsed_.lock();
	is_elapsed_=false;
	mutex_is_elapsed_.unlock();


	double begin_time=ros::Time::now().toSec();
	bool should_quit=false;
	
	while (!should_quit) {
		double now=ros::Time::now().toSec();
		if (now-begin_time>seconds_to_wait_) {
			mutex_is_elapsed_.lock();
			is_elapsed_=true;
			mutex_is_elapsed_.unlock();
			should_quit=true;
		}
		else {
			mutex_stop_.lock();
			if (stop_==true) {
				should_quit=true;
			}
			mutex_stop_.unlock();
		}
	}
}

bool SupervisionTimer::isElapsed() {
	boost::lock_guard<boost::mutex> lock(mutex_is_elapsed_);
	return is_elapsed_;
}