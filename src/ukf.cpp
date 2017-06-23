#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <utility>
#include <tuple>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::pair;
using std::tuple;
using std::get;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() :
		isInitialised { false }, previousTimeStamp { 0 }, x_n { 0 }, xAug_n { 0 } {
	// if this is false, laser measurements will be ignored (except during init)
	useLaser = true;

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
	//P *= 100.;
	//P.topLeftCorner(2, 2).setIdentity();

	// Initialize the number of state components, and augmented state components

	x_n = x.size();
	xAug_n = x_n + 2;

	std_a = 1.;
	std_yawdd = 1.;

	//radar measurement noise standard deviation radius in m
	std_radr = 0.3;

	//radar measurement noise standard deviation angle in rad
	std_radphi = 0.0175;

	//radar measurement noise standard deviation radius change in m/s
	std_radrd = 0.1;

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
		cout << "x=" << endl << x << endl;
		cout << "P=" << endl << P << endl;
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
	double deltaT = (meas_package.timeStamp - previousTimeStamp) / 1000000.0; // seconds
	cout << "Delta-t= " << deltaT << endl;

	// Prediction

	MatrixXd XsigAug = computeAugmentedSigmaPoints();
	cout << "XsigAug=" << endl << XsigAug << endl;
	MatrixXd XsigPred = predictSigmaPoints(XsigAug, deltaT);
	cout << "XsigPred=" << endl << XsigPred << endl;
	predictStateAndCovariance(XsigPred);
	cout << "x=" << endl << x << endl;
	cout << "P=" << endl << P << endl;

	// Update

	auto predictedMeasurements = (meas_package.sensorType == MeasurementPackage::RADAR)?
			predictRadarMeasurments(XsigPred):
			predictLidarMeasurments(XsigPred);
	VectorXd zPred { get<0>(predictedMeasurements) };
	MatrixXd Zsig { get<1>(predictedMeasurements) };
	MatrixXd S { get<2>(predictedMeasurements) };
	updateStateWithMeasurements(meas_package, zPred, Zsig, S, XsigPred);

	// Set previousTimeStamp for the next iteration
	previousTimeStamp = meas_package.timeStamp;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

MatrixXd UKF::computeSigmaPoints() {
	/*
	 *Generate sigma points, given state dimension, lambda, current state and current covariance matrix
	 */

	// Get the square root of the covariance
	MatrixXd A { P.llt().matrixL() };

	// Now the sigma points

	MatrixXd Xsig { MatrixXd(x_n, 2 * x_n + 1) };

	auto lambda = 3 - x_n;

	auto spread { sqrt(lambda + x_n) };
	Xsig.col(0) = x;
	for (auto iCol = 0; iCol < x_n; ++iCol) {
		Xsig.col(iCol + 1) = x + spread * A.col(iCol);
		Xsig.col(iCol + 1 + x_n) = x - spread * A.col(iCol);
	}

	return Xsig;
}

MatrixXd UKF::computeAugmentedSigmaPoints() {
	/* Determine augmented sigma points based on current state, covariance, noise standard deviation for
	 * longitudinal acceleration and yaw acceleration and lambda
	 */

	auto lambda = 3 - xAug_n;
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

	return XsigAug;

}

MatrixXd UKF::predictSigmaPoints(const MatrixXd & XsigAug, double deltaT) {
	/*
	 * Predict sigma points based on augmented sigma points.
	 */

	double epsilon = 0.000001;
	//create matrix with predicted sigma points as columns
	auto XsigPred = MatrixXd(x_n, 2 * xAug_n + 1);

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
		if (abs(psiDot) < epsilon)
			a << v * cos(psi) * deltaT, v * sin(psi) * deltaT, 0, psiDot
					* deltaT, 0;
		else
			a << v / psiDot * (sin(psi + psiDot * deltaT) - sin(psi)), v
					/ psiDot * (-cos(psi + psiDot * deltaT) + cos(psi)), 0, psiDot
					* deltaT, 0;
		XsigPred.col(i) = XsigAug.block(0, i, 5, 1) + a + b;
	}

	return XsigPred;

}

pair<MatrixXd, MatrixXd> UKF::predictStateAndCovariance(
		const MatrixXd & XsigPred) {
	/*
	 * Predict expected state and covariance, and determine weights, based on predicted sigma points
	 */

	auto lambda = 3. - xAug_n;

	auto weights = VectorXd(2 * xAug_n + 1);
	weights.setZero();
	weights(0) = lambda / (lambda + xAug_n);  // Set the first component
	weights.bottomRows(2 * xAug_n).setConstant(1 / (2 * (lambda + xAug_n))); // Set the remaining 2*n_aug components

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

	auto ret = make_pair(x, P);
	return ret;

}

pair<VectorXd, MatrixXd> UKF::predictMeasurements(const MatrixXd & Zsig,
		const MatrixXd & R) {

	//define spreading parameter
	double lambda = 3 - xAug_n;  // TODO same as in predictSateAndCovariance()

	auto weights = VectorXd(2 * xAug_n + 1);
	weights.setZero();
	weights(0) = lambda / (lambda + xAug_n);  // Set the first component
	weights.bottomRows(2 * xAug_n).setConstant(1 / (2 * (lambda + xAug_n))); // Set the remaining 2*n_aug components

	auto z_n = Zsig.rows();
	// Predict measurement, expected value
	VectorXd zPred = VectorXd(z_n);
	zPred.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i)
		zPred += weights(i) * Zsig.col(i);

	// Predicted measurement covariance
	auto S = MatrixXd(z_n, z_n);
	S.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		VectorXd zDiff = Zsig.col(i) - zPred;
		zDiff(1) = normaliseAngle(zDiff(1));
		S += weights(i) * (zDiff * zDiff.transpose());
	}

	S += R;

	//cout << "S="<< endl << S << endl;
	return make_pair(zPred, S);

}

tuple<VectorXd, MatrixXd, MatrixXd> UKF::predictRadarMeasurments(
		const MatrixXd & XsigPred) {
	int z_n = 3; // Number of components in radar measurments

	// Sigma points in measurement space
	MatrixXd Zsig = MatrixXd(z_n, 2 * xAug_n + 1);

	// Transform sigma points into measurement space, implementing the measurement model
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

	// Radar noise covariance TODO move it to init()
	auto R = MatrixXd(3, 3);
	R << std_radr * std_radr, 0, 0, 0, std_radphi * std_radphi, 0, 0, 0, std_radrd
			* std_radrd;

	auto predicted = predictMeasurements(Zsig, R);
	auto zPred = predicted.first;
	auto S = predicted.second;

	return make_tuple(zPred, Zsig, S);
}


tuple<VectorXd, MatrixXd, MatrixXd> UKF::predictLidarMeasurments(
		const MatrixXd & XsigPred) {
	int z_n = 2; // Number of components in lidar measurments

	// Sigma points in measurement space
	MatrixXd Zsig = MatrixXd(z_n, 2 * xAug_n + 1);

	// Transform sigma points into measurement space, implementing the measurement model
	Zsig.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		auto px = XsigPred(0, i);
		auto py = XsigPred(1, i);
		Zsig.col(i) << px, py;
	}

	// Radar noise covariance TODO move it to init()
	auto R = MatrixXd(2, 2);
	R << std_laspx * std_laspx, 0, 0, std_laspy * std_laspy;

	auto predicted = predictMeasurements(Zsig, R);
	auto zPred = predicted.first;
	auto S = predicted.second;

	return make_tuple(zPred, Zsig, S);
}


pair<VectorXd, MatrixXd> UKF::updateStateWithMeasurements(
		const MeasurementPackage & meas_package, const VectorXd & zPred,
		const MatrixXd & Zsig, const MatrixXd & S, const MatrixXd & XsigPred) {
	//create matrix for cross correlation Tc
	VectorXd z { meas_package.rawMeasurement };
	auto z_n = z.size();
	MatrixXd Tc = MatrixXd(x_n, z_n);

	auto lambda = 3. - xAug_n;  // TODO fix code duplication

	auto weights = VectorXd(2 * xAug_n + 1);
	weights.setZero();
	weights(0) = lambda / (lambda + xAug_n);  // Set the first component
	weights.bottomRows(2 * xAug_n).setConstant(1 / (2 * (lambda + xAug_n))); // Set the remaining 2*n_aug components

	//calculate cross correlation matrix
	Tc.setZero();
	for (auto i = 0; i < 2 * xAug_n + 1; ++i) {
		VectorXd x_diff = XsigPred.col(i) - x;
		x_diff(3) = normaliseAngle(x_diff(3));
		VectorXd z_diff = Zsig.col(i) - zPred;
		if (z_diff.size()==3)  // If z_dif has 3 dimensions then it comes from radar, and angle theta needs to be normalised
			z_diff(1) = normaliseAngle(z_diff(1));
		Tc += weights(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K { Tc * S.inverse() };

	//update state mean and covariance matrix

	VectorXd z_diff = z - zPred;
	z_diff(1) = normaliseAngle(z_diff(1));
	x += K * z_diff;
	P -= K * S * K.transpose();

	return make_pair(x, P);

}
void UKF::test() {

	auto tolerance = 0.000001; // Tolerance in matrices comparison for test purpose
	MeasurementPackage initMeasurement;
	initMeasurement.sensorType = MeasurementPackage::RADAR;
	initMeasurement.timeStamp = 1477010443050000;
	VectorXd rawInit(3);
	rawInit << 1.014892e+00, 5.543292e-01, 4.892807e+00;
	initMeasurement.rawMeasurement = rawInit;

	init(initMeasurement);

	P.setIdentity();
	P *= 1000.;
	P.topLeftCorner(2, 2).setIdentity();
	x_n = x.size();
	xAug_n = x_n + 2;
	std_a = .2;
	std_yawdd = .2;
	std_radr = 0.3;
	std_radphi = 0.0175;
	std_radrd = 0.1;

	x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

	//set example covariance matrix
	P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011, 0.0071, 0.0060, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071, 0.0007, 0.0098, 0.0100, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

	std_a = .2;
	std_yawdd = .2;

	MatrixXd Xsig = computeSigmaPoints();
	MatrixXd compareWith(5, 11);
	compareWith << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879;
	assert(Xsig.isApprox(compareWith, tolerance));

	MatrixXd XsigAug = computeAugmentedSigmaPoints();
	compareWith = MatrixXd(7, 15);
	// cout << "XsigAug=" << endl << XsigAug << endl;
	compareWith << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38, 2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049, 0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015, 0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0, 0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;
	assert(XsigAug.isApprox(compareWith, tolerance));

	double deltaT { .1 };

	MatrixXd XsigPred = predictSigmaPoints(XsigAug, deltaT);
	// cout << "XsigPred=" << endl << XsigPred << endl;
	compareWith = MatrixXd(5, 15);
	compareWith << 5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553, 1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939, 2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049, 0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048, 0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
	assert(XsigPred.isApprox(compareWith, tolerance));

	auto stateAndCovariance = predictStateAndCovariance(XsigPred);
	// cout << "x=" << endl << x << endl;
	VectorXd compareWithV(5);
	compareWithV << 5.93446, 1.48886, 2.2049, 0.53678, 0.3528;
	assert(stateAndCovariance.first.isApprox(compareWithV, tolerance));
	// cout << "P=" << endl << P << endl;
	compareWith = MatrixXd(5, 5);
	compareWith << 0.00548035, -0.002499, 0.00340508, -0.00357408, -0.0030908, -0.002499, 0.0110543, 0.00151778, 0.00990746, 0.00806631, 0.00340508, 0.00151778, 0.0058, 0.00078, 0.0008, -0.00357408, 0.00990746, 0.00078, 0.011924, 0.01125, -0.0030908, 0.00806631, 0.0008, 0.01125, 0.0127;
	assert(stateAndCovariance.second.isApprox(compareWith, tolerance));

	auto predictedMeasurements = predictRadarMeasurments(XsigPred);
	VectorXd zPred { get<0>(predictedMeasurements) };
	MatrixXd Zsig { get<1>(predictedMeasurements) };
	MatrixXd S { get<2>(predictedMeasurements) };
	// cout << "zPred=" << endl << zPred << endl;
	compareWithV = VectorXd(3);
	compareWithV << 6.11934, 0.245834, 2.10274;
	assert(zPred.isApprox(compareWithV, tolerance));

	// cout << "Zsig=" << endl << Zsig << endl;
	compareWith = MatrixXd(3, 15);
	compareWith << 6.11954, 6.23274, 6.15173, 6.12724, 6.11254, 6.11934, 6.12122, 6.11954, 6.00666, 6.08806, 6.11171, 6.12448, 6.11974, 6.11787, 6.11954, 0.245851, 0.234255, 0.274046, 0.246849, 0.249279, 0.245965, 0.245923, 0.245851, 0.257693, 0.217355, 0.244896, 0.242332, 0.245737, 0.24578, 0.245851, 2.11225, 2.21914, 2.06474, 2.18799, 2.03565, 2.1081, 2.14548, 2.11115, 2.00221, 2.13, 2.03506, 2.16622, 2.1163, 2.07902, 2.11334;
	assert(Zsig.isApprox(compareWith, tolerance));
	/*
	 * S=
	 0.0946302, -0.000145123,   0.00408742,
	 -0.000145123,  0.000624209, -0.000781362,
	 0.00408742, -0.000781362,    0.0180473;
	 */

	MeasurementPackage meas_package;
	meas_package.rawMeasurement = VectorXd(3);
	meas_package.rawMeasurement << 5.9214, 0.2187, 2.0062;
	meas_package.sensorType = MeasurementPackage::RADAR;
	meas_package.timeStamp = 1477010443100000;

	/*auto S { MatrixXd(3, 3) };
	 S << 0.0946302, -0.000145123,   0.00408742,
	 -0.000145123,  0.000624209, -0.000781362,
	 0.00408742, -0.000781362,    0.0180473;*/
	auto updatedState = updateStateWithMeasurements(meas_package, zPred,
			Zsig, S, XsigPred);
	// cout << "x=" << endl << updatedState.first << endl;
	compareWithV = VectorXd(5);
	compareWithV << 5.92115, 1.41666, 2.15551, 0.48931, 0.31995;
	assert(updatedState.first.isApprox(compareWithV, tolerance));

	// cout << "P=" << endl << updatedState.second << endl;
	compareWith = MatrixXd(5, 5);
	compareWith << 0.00362505, -0.000375919, 0.00207001, -0.000983428, -0.000769897, -0.000375919, 0.0054474, 0.00158839, 0.00454767, 0.00361869, 0.00207001, 0.00158839, 0.00409776, 0.00158566, 0.00170133, -0.000983428, 0.00454767, 0.00158566, 0.00647923, 0.00662974, -0.000769897, 0.00361869, 0.00170133, 0.00662974, 0.0087481;
	assert(updatedState.second.isApprox(compareWith, tolerance));

	cout << "Unit test passed!" << endl;
}

