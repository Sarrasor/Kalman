#ifndef LIDAR_TRACKER_H_
#define LIDAR_TRACKER_H_

#include "Measurement.h"
#include "KalmanFilter.h"
#include <vector>
#include <string>
#include <fstream>

class LidarTracker 
{
public:
	LidarTracker();
	virtual ~LidarTracker();
	void ProcessMeasurement(const Measurement &measurement);
	KalmanFilter kf_;

private:
	bool is_initialized_;
	int64_t previous_timestamp_;

	// Acceleration noise components
	float noise_ax;
	float noise_ay;
};

#endif /* LIDAR_TRACKER_H_ */