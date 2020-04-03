#include <iostream>
#include "Dense"
#include "LidarTracker.h"
#include "Measurement.h"
#include <vector>

// How many measurements to process
#define N_MEASUREMENTS 10

using namespace std;
using namespace Eigen;

vector<Measurement> measurements;

int main(int argc, char const *argv[])
{

	// Hardcoded input file with laser and radar measurements
	string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

	if (!in_file.is_open()) 
	{
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;
	// Set i to get only first N_MEASUREMENTS
	int i = 0;
	while(getline(in_file, line) && (i <= N_MEASUREMENTS))
	{
		Measurement measurement;

		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type;	// Reads first element from the current line
		int64_t timestamp;

		if(sensor_type.compare("L") == 0)
		{
			// Read Lidar measurements
			measurement.sensor_type_ = Measurement::LASER;
			measurement.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			measurement.raw_measurements_ << x, y;
			iss >> timestamp;
			measurement.timestamp_ = timestamp;
			measurements.push_back(measurement);
		}
		else if(sensor_type.compare("R") == 0)
		{
			// Skip Radar measurements
			continue;
		}
		i++;
	}

	//Create a LidarTracker instance
	LidarTracker tracker;

	// Call the ProcessMeasurement() function for each measurement
	size_t N = measurements.size();
	for (size_t i = 0; i < N; ++i) 
	{
		// Tracker starts filtering from the second measurement
		// because the velocity is initially unknown
		tracker.ProcessMeasurement(measurements[i]);
	}

	// Close the input file
	if(in_file.is_open())
	{
		in_file.close();
	}
	
	return 0;
}