#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "Dense"

class Measurement
{
public:
	int64_t timestamp_;

	enum SensorType {
		LASER, RADAR
	} sensor_type_;
	
	Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_H_ */