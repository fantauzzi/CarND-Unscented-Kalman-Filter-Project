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
	 * Sets initial state (x) and its covariance (P) based on given measurement. For initialisation only, the given
	 * measurement is used whether it is radar or lidar, even if the UKF was directed to ignore radar
	 * or lidar measurements.
	 * Also sets the number of components in state and augmented state vector (x_n and xAug_n respectively);
	 * sets the standard deviation for acceleration and yaw acceleration (std_a and std_yawdd); sets the
	 * standard deviation for radar measurements (std_radr, std_radphi, std_radrd); initializes the weights
	 * (weights); sets isInitialised to True.
	 */
	void init(MeasurementPackage meas_package);

	// True iff processMeasurement() has been called at least once on this istance, and therefore init() has been called
	bool isInitialised;

	// If set to false, laser measurements will be ignored (except for initialisation)
	bool useLaser;

	// if set to false, radar measurements will be ignored (except for initialisation)
	bool useRadar;

	// State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x;

	// State covariance matrix
	MatrixXd P;

	/* Weights entering the prediction of the state and its covariance, the prediction of measurements
	and their covariance, and calculation of the cross-correlation matrix. */
	VectorXd weights;

	// The NIS
	double nis;

	long long previousTimeStamp;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	const double std_a;

	// Process noise standard deviation yaw acceleration in rad/s^2
	const double std_yawdd;

	// Laser measurement noise standard deviation position1 in m
	const double std_laspx;

	// Laser measurement noise standard deviation position2 in m
	const double std_laspy;

	// Radar measurement noise standard deviation radius in m
	const double std_radr;

	// Radar measurement noise standard deviation angle in rad
	const double std_radphi;

	// Radar measurement noise standard deviation radius change in m/s
	const double std_radrd;

	// Number of dimensions in state vector
	const int x_n;

	// Number of dimension in augmnented state vector
	const int xAug_n;

	/* Compute the sigma points of the non-augmented state vector; useful for testing, not
	 * actually used by the UKF.
	 */
	MatrixXd computeSigmaPoints();

	/* Determine augmented sigma points based on current state, covariance, noise standard deviation for
	 * longitudinal acceleration and yaw acceleration and lambda
	 */
	MatrixXd computeAugmentedSigmaPoints();

	/*
	 * Predict sigma points based on augmented sigma points.
	 */
	MatrixXd predictSigmaPoints(const MatrixXd & XsigAug, double deltaT);

	/*
	 * Predict expected state and covariance based on predicted sigma points
	 */
	pair<MatrixXd, MatrixXd> predictStateAndCovariance(const MatrixXd & XsigPred);

	pair<VectorXd, MatrixXd>  predictMeasurements(const MatrixXd & Zsig, const MatrixXd & R);

	tuple<VectorXd, MatrixXd, MatrixXd>  predictRadarMeasurments(const MatrixXd & XsigPred);

	tuple<VectorXd, MatrixXd, MatrixXd>  predictLidarMeasurments(const MatrixXd & XsigPred);

	tuple<VectorXd, MatrixXd, double> updateStateWithMeasurements(const MeasurementPackage & meas_package, const VectorXd & zPred, const MatrixXd & Zsig, const MatrixXd & S, const MatrixXd & XsigPred);

public:
	UKF();

	virtual ~UKF();

	// Execute unit-test
	void test();

	VectorXd getState() const;

	double getNIS() const;

	/**
	 * ProcessMeasurement
	 * @param meas_package The latest measurement data of either radar or laser
	 */
	void processMeasurement(MeasurementPackage meas_package);
};
