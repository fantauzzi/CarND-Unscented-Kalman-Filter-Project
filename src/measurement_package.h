#pragma once
#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timeStamp;

  enum SensorType{
    LIDAR,
    RADAR
  } sensorType;

  Eigen::VectorXd rawMeasurement;

};
