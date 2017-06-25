#pragma once
#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <utility>
#include <tuple>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::pair;
using std::tuple;

class UKF {
	/*
	 * Set initial state and its covariance based on given measurement. For initialisation only, the given
	 * measurement will be used whether it is radar or lidar, even if the object was directed to ignore radar
	 * or lidar measurements.
	 */
	void init(MeasurementPackage meas_package);

	///* initially set to false, set to true in first call of ProcessMeasurement
	bool isInitialised;

	///* if this is false, laser measurements will be ignored (except for init)
	bool useLaser;

	///* if this is false, radar measurements will be ignored (except for init)
	bool useRadar;

	///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x;

	///* state covariance matrix
	MatrixXd P;

	VectorXd weights;

	double nis;

	long long previousTimeStamp;

	///* Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a;

	///* Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd;

	///* Laser measurement noise standard deviation position1 in m
	double std_laspx;

	///* Laser measurement noise standard deviation position2 in m
	double std_laspy;

	///* Radar measurement noise standard deviation radius in m
	double std_radr;

	///* Radar measurement noise standard deviation angle in rad
	double std_radphi;

	///* Radar measurement noise standard deviation radius change in m/s
	double std_radrd;

	///* Weights of sigma points
	// VectorXd weights;

	///* Sigma point spreading parameter
	// double lambda;

	///* State dimension
	int x_n;

	///* Augmented state dimension
	int xAug_n;

	// bool debug;

	MatrixXd computeSigmaPoints();

	MatrixXd computeAugmentedSigmaPoints();

	MatrixXd predictSigmaPoints(const MatrixXd & XsigAug, double deltaT);

	pair<MatrixXd, MatrixXd> predictStateAndCovariance(const MatrixXd & XsigPred);

	pair<VectorXd, MatrixXd>  predictMeasurements(const MatrixXd & Zsig, const MatrixXd & R);

	tuple<VectorXd, MatrixXd, MatrixXd>  predictRadarMeasurments(const MatrixXd & XsigPred);

	tuple<VectorXd, MatrixXd, MatrixXd>  predictLidarMeasurments(const MatrixXd & XsigPred);

	tuple<VectorXd, MatrixXd, double> updateStateWithMeasurements(const MeasurementPackage & meas_package, const VectorXd & zPred, const MatrixXd & Zsig, const MatrixXd & S, const MatrixXd & XsigPred);

public:
	/**
	 * Constructor
	 */
	UKF();

	/**
	 * Destructor
	 */
	virtual ~UKF();

	void test();

	VectorXd getState() const;

	double getNIS() const;

	/**
	 * ProcessMeasurement
	 * @param meas_package The latest measurement data of either radar or laser
	 */
	void processMeasurement(MeasurementPackage meas_package);

	/**
	 * Prediction Predicts sigma points, the state, and the state covariance
	 * matrix
	 * @param delta_t Time between k and k+1 in s
	 */
};
