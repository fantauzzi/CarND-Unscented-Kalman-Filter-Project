#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() :
		isInitialised { false }, previousTimeStamp { 0 }, lambda { .0 }, x_n { 0 }, xAug_n {
				0 } {
	// if this is false, laser measurements will be ignored (except during init)
	useLaser = false;

	// if this is false, radar measurements will be ignored (except during init)
	useRadar = true;

	// initial state vector
	x = VectorXd(5);

	// initial covariance matrix
	P = MatrixXd(5, 5);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd = 30;

	// Laser measurement noise standard deviation position1 in m
	std_laspx = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd = 0.3;

	/**
	 TODO:

	 Complete the initialization. See ukf.h for other member properties.

	 Hint: one or more values initialized above might be wildly off...
	 */
}

UKF::~UKF() {
}

void UKF::init(MeasurementPackage meas_package) {
	// Initialise current state
	if (meas_package.sensorType == MeasurementPackage::LIDAR) {
		auto px = meas_package.rawMeasurement(0);
		auto py = meas_package.rawMeasurement(1);
		auto v = 1.;
		auto psi = 0.;
		auto psiDot = 0.;
		x << px, py, v, psi, psiDot;
	} else {
		auto rho = meas_package.rawMeasurement(0);
		double theta = meas_package.rawMeasurement(1); // enforced to double to circumvent Eclipse CDT parser bug
		auto rhoDot = meas_package.rawMeasurement(2);
		auto px = rho * cos(theta);
		auto py = rho * sin(theta);
		auto v = rhoDot;  // Incorrect, but OK for initialisation
		auto psi = theta;  // Incorrect, but OK for initialisation
		auto psiDot = 0.;
		x << px, py, v, psi, psiDot;
	}

	// Initialise current state covariance
	P.setIdentity();
	P *= 1000.;
	P.topLeftCorner(2, 2).setIdentity();

	// Initialize the number of state components, and augmented state components

	x_n = x.size();
	xAug_n = x_n + 2;

	// Initialise sigma points spreading parameter
	lambda = 3. - x.size();

	weights = VectorXd(2 * xAug_n + 1);
	weights.setZero();
	weights(0) = lambda / (lambda + xAug_n);  // Set the first component
	weights.bottomRows(2 * xAug_n).setConstant(1 / (2 * (lambda + xAug_n))); // Set the remaining 2*n_aug components

	// Chop chop, job done!
	isInitialised = true;
}

VectorXd UKF::getState() const {
	return x;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::processMeasurement(MeasurementPackage meas_package) {
	// Handle initialisation
	if (!isInitialised) {
		init(meas_package);
		// Set previousTimeStamp for the next iteration
		previousTimeStamp = meas_package.timeStamp;
		return; // That's it! After initialisation, nothing more to do until another measurement is collected.
	}

	/* If the measurement is from an instrument to be ignored, do nothing and just return.
	 * Note: no update to previousTimeStamp at this time.
	 */

	if (!useLaser && meas_package.sensorType == MeasurementPackage::LIDAR)
		return;

	if (!useRadar && meas_package.sensorType == MeasurementPackage::RADAR)
		return;

	// Calculate the time elapsed between the previous measurement and the current one, in seconds.
	double deltaT = (meas_package.timeStamp - previousTimeStamp) / 1000000; // seconds

	prediction(deltaT);

	updateRadar(meas_package);

	// Set previousTimeStamp for the next iteration
	previousTimeStamp = meas_package.timeStamp;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

void UKF::prediction(double deltaT) {
	/*
	 *Generate sigma points, given state dimension, lambda, current state and current covariance matrix
	 */

	// Get the square root of the covariance
	MatrixXd A { P.llt().matrixL() };

	// Now the sigma points

	MatrixXd Xsig { MatrixXd(x.size(), 2 * x.size() + 1) };

	auto spread { sqrt(lambda + x_n) };
	Xsig.col(0) = x;
	for (auto iCol = 0; iCol < x_n; ++iCol) {
		Xsig.col(iCol + 1) = x + spread * A.col(iCol);
		Xsig.col(iCol + 1 + x_n) = x - spread * A.col(iCol);
	}

	/* Determine augmented sigma points based on current state, covariance, noise standard deviation for
	 * longitudinal acceleration and yaw acceleration and lambda
	 */

	// The augmented state
	VectorXd xAug = VectorXd(xAug_n);
	xAug.setZero();
	xAug.head(x_n) = x;

	// Covariance matrix for acceleration and yaw acceleration errors
	auto Q = MatrixXd(2, 2);
	Q << pow(std_a, 2), 0, 0, pow(std_yawdd, 2);

	// The augmented state covariance
	MatrixXd P_Aug = MatrixXd(xAug_n, xAug_n);
	P_Aug.setZero();
	P_Aug.block(0, 0, x_n, x_n) = P;
	P_Aug.block(x_n, x_n, 2, 2) = Q;

	// Get square root of augmented state covariance
	MatrixXd A_Aug = P_Aug.llt().matrixL();

	//Calculate augmented sigma points
	MatrixXd XsigAug = MatrixXd(xAug_n, 2 * xAug_n + 1);
	auto spreadAug = sqrt(lambda + xAug_n);
	XsigAug.col(0) = xAug;
	for (auto iCol = 0; iCol < xAug_n; ++iCol) {
		XsigAug.col(iCol + 1) = xAug + spreadAug * A_Aug.col(iCol);
		XsigAug.col(iCol + 1 + xAug_n) = xAug - spreadAug * A_Aug.col(iCol);
	}

	/*
	 * Predict sigma points based on augmented sigma points.
	 */

	//create matrix with predicted sigma points as columns
	XsigPred = MatrixXd(x_n, 2 * xAug_n + 1);

	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		auto v = XsigAug(2, i);
		auto psi = XsigAug(3, i);
		auto psiDot = XsigAug(4, i);
		auto nu_a = XsigAug(5, i);
		auto nu_psiDotDot = XsigAug(6, i);
		VectorXd b { VectorXd(5) };
		b << .5 * pow(deltaT, 2) * cos(psi) * nu_a, .5 * pow(deltaT, 2)
				* sin(psi) * nu_a, deltaT * nu_a, .5 * pow(deltaT, 2)
				* nu_psiDotDot, deltaT * nu_psiDotDot;
		VectorXd a { VectorXd(5) };
		if (psiDot == 0)
			a << v * cos(psi) * deltaT, v * sin(psi) * deltaT, 0, psiDot
					* deltaT, 0;
		else
			a << v / psiDot * (sin(psi + psiDot * deltaT) - sin(psi)), v
					/ psiDot * (-cos(psi + psiDot * deltaT) + cos(psi)), 0, psiDot
					* deltaT, 0;
		XsigPred.col(i) = XsigAug.block(0, i, 5, 1) + a + b;
	}

	/*
	 * Predict expected state and covariance, and determine weights, based on predicted sigma points
	 */

	// Predict state expected value
	x.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i)
		x += weights(i) * XsigPred.col(i);

	// Covariance of the predicted state
	P.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; i++) {  //iterate over sigma points
		// State difference
		VectorXd x_diff = XsigPred.col(i) - x;
		// Angle normalisation
		x_diff(3) = normaliseAngle(x_diff(3));
		P += weights(i) * x_diff * x_diff.transpose();
	}

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::updateLidar(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Use lidar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the lidar NIS.
	 */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::updateRadar(MeasurementPackage meas_package) {

	/* Predict radar and lidar measurements and their covariances based on lambda, weights (same as before? check!),
	 * noise standard deviations (on each measurement component), predicted sigma points,
	 */

	int z_n = 3; // Number of components in radar measurments

	// Sigma points in measurement space
	MatrixXd Zsig = MatrixXd(z_n, 2 * xAug_n + 1);

	//transform sigma points into measurement space
	Zsig.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		auto px = XsigPred(0, i);
		auto py = XsigPred(1, i);
		auto v = XsigPred(2, i);
		auto psi = XsigPred(3, i);
		auto rho = sqrt(px * px + py * py);
		auto phi = atan2(py, px);
		auto rhoDot = (px * cos(psi) * v + py * sin(psi) * v) / rho;
		Zsig.col(i) << rho, phi, rhoDot;
	}

	// Predict measurement, expected value
	VectorXd zPred = VectorXd(z_n);
	zPred.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i)
		zPred += weights(i) * Zsig.col(i);

	// Predicted measurement covariance
	S = MatrixXd(z_n, z_n);
	S.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		VectorXd col = Zsig.col(i) - zPred;
		S += weights(i) * (col * col.transpose());
	}

	// Radar noise covariance TODO move it to init()
	auto R = MatrixXd(3, 3);
	R << std_radr * std_radr, 0, 0, 0, std_radphi * std_radphi, 0, 0, 0, std_radrd
			* std_radrd;
	S += R;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(x_n, z_n);

	//calculate cross correlation matrix
	Tc.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		VectorXd x_diff = XsigPred.col(i) - x;
		x_diff(1) = normaliseAngle(x_diff(1));
		VectorXd z_diff = Zsig.col(i) - zPred;
		z_diff(1) = normaliseAngle(z_diff(1));
		Tc += weights(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K { Tc * S.inverse() };

	//update state mean and covariance matrix

	VectorXd z { meas_package.rawMeasurement };

	VectorXd z_diff = z - zPred;
	z_diff(1) = normaliseAngle(z_diff(1));
	x += K * z_diff;
	P -= K * S * K.transpose();

}
