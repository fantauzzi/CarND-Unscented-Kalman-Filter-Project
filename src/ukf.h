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
	/**
	 */
	/**
	 * Sets initial state (data member x) and its covariance (data member P) based on given measurement.
	 * Because it is initialisation, the given measurement is used whether it is radar or lidar, even if
	 * the UKF was directed to ignore radar or lidar measurements.
	 * Sets the isInitialised data memeber to True.
	 * @param meas_package the latest measurements collected from either lidar or radar.
	 */
	void init(MeasurementPackage meas_package);

	// True iff processMeasurement() has been called at least once on this istance, and therefore init() has been called
	bool isInitialised;

	// If set to false, laser measurements will be ignored (except for initialisation)
	bool useLidar;

	// if set to false, radar measurements will be ignored (except for initialisation)
	bool useRadar;

	// State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd x;

	// State covariance matrix
	MatrixXd P;

	/* Weights entering the prediction of the state and its covariance, the prediction of measurements
	and their covariance, and calculation of the cross-correlation matrix. */
	VectorXd weights;

	// The last computed NIS (defaults to 0 if not yet computed)
	double nis;

	// Time-stamp of the last update; in microseconds from an epoch
	long long previousTimeStamp;

	// Process noise standard deviation longitudinal acceleration in m/s^2
	const double std_a;

	// Process noise standard deviation yaw acceleration in rad/s^2
	const double std_yawdd;

	// Laser measurement noise standard deviation position x in m
	const double std_laspx;

	// Laser measurement noise standard deviation position y in m
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

	/**
	 * Computes the sigma points of the non-augmented state vector; useful for testing, not
	 * actually used by the UKF.
	 * @return a amtrix whose columns are the sigma points.
	 */
	MatrixXd computeSigmaPoints() const;

	/**
	 * Determines the augmented sigma points based on current state, covariance, noise standard
	 * deviation for longitudinal acceleration and yaw acceleration.
	 * @return A matrix whose columns are the augmented sigma points
	 */
	MatrixXd computeAugmentedSigmaPoints() const;

	/**
	 * Predicts values for given augmented sigma points after a time interval.
	 * @param XsigAug the augmented sigma points, at the beginning of the time interval.
	 * @param deltaT the time interval.
	 * @return a matrix whose columns are the predicted sigma points.
	 */
	MatrixXd predictSigmaPoints(const MatrixXd & XsigAug, double deltaT) const;

	/**
	 * Determines the expected state and covariance based on given predicted sigma points.
	 * @param XsigPred the predicted sigma points. The method updates the
	 * object data members x and P.
	 * @return a pair whose first element is the expected state vector, and second element
	 * is the covariance matrix.
	 */
	pair<VectorXd, MatrixXd> predictStateAndCovariance(const MatrixXd & XsigPred);

	/**
	 * Determines the expected value of the next measurements, based on given predicted sigma
	 * points in measurements space, and the sensor noise covariance.
	 * @param Zsig a matrix whose columns are the predicted sigma points, expressed in
	 * the measurements space.
	 * @param R the sensor noise covariance matrix.
	 * @return a pair whose first element is a vector with the measurements expected value, and
	 * whose second element is its covariance matrix.
	 */
	pair<VectorXd, MatrixXd>  predictMeasurements(const MatrixXd & Zsig, const MatrixXd & R) const;

	/**
	 * Determines the expected value of the next radar measurements, based on given predicted
	 * sigma points.
	 * @param XsigPred a matrix whose columns are the predicted sigma points.
	 * @return a triple: the first element is the vector of measurements expected values;
	 * the second element is a matrix whose columns are the sigma points in measurements
	 * space; the third element is the expected values covariance matrix.
	 */
	tuple<VectorXd, MatrixXd, MatrixXd>  predictRadarMeasurments(const MatrixXd & XsigPred) const;

	/**
	 * Determines the expected value of the next lidar measurements, based on given predicted
	 * sigma points.
	 * @param XsigPred a matrix whose columns are the predicted sigma points.
	 * @return a triple: the first element is the vector of measurements expected values;
	 * the second element is a matrix whose columns are the sigma points in measurements
	 * space; the third element is the expected values covariance matrix.
	 */
	tuple<VectorXd, MatrixXd, MatrixXd>  predictLidarMeasurments(const MatrixXd & XsigPred) const;

	/**
	 * Updates the currently predicted state x, and its covariance P, based on the latest sensor measurments.
	 * Also computes the NIS. The method updates the object data members x, P and nis.
	 * @param meas_package the latest sensor measurements, to be used for update.
	 * @param zPred the measurements expected value (predicted value), as returned by predictLidarMeasurments() or
	 * predictRadarMeasurments().
	 * @param Zsig the sigma points in measurements space, as returned by predictLidarMeasurments() or
	 * predictRadarMeasurments().
	 * @param S the covariance for the measurements expected value (predicted value), as returned by
	 * predictLidarMeasurments() or predictRadarMeasurments().
	 * @param XsigPred the predicted sigma points, as returned by predictSigmaPoints().
	 * @return a triple: the first element is the updated state vector; the second element is the updated state
	 * covariance; the third is the computed NIS.
	 */
	tuple<VectorXd, MatrixXd, double> updateStateWithMeasurements(const MeasurementPackage & meas_package, const VectorXd & zPred, const MatrixXd & Zsig, const MatrixXd & S, const MatrixXd & XsigPred);

public:
	UKF();

	UKF(const bool useRadar, const bool useLidar);

	virtual ~UKF();

	/**
	 * Fetches the filter current state.
	 * @return a copy of the state vector.
	 */
	VectorXd getState() const;

	/**
	 * Fetches the filter current state covariance.
	 * @return a copy of the covariance matrix.
	 */
	MatrixXd getCovariance() const;

	/**
	 * Fetches the latest computed NIS value, 0 if never computed.
	 * @return the latest computed NIS value.
	 */
	double getNIS() const;

	/**
	 * Process the given measurements in the Unscented Kalman Filter, and updates the object
	 * data members x, P and nis. Result of the processing can be retrieved with subsequent
	 * calls to methods getState() and getNIS().
	 * @param meas_package The latest measurement data coming from either radar or lidar.
	 */
	void processMeasurement(MeasurementPackage meas_package);

	/**
	 * Execute unit-test, for debugging.
	 */
	void test();
};
