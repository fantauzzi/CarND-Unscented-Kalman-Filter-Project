#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &groundTruth) {
	VectorXd rmse(4);
	rmse.setZero();
	// Check the validity of the input parameters
	if (estimations.size() != groundTruth.size() || estimations.size() == 0) {
		cout
		<< "ERROR calculateRMSE() - Invalid estimation or groundTruth data."
		<< endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - groundTruth[i];
		residual = residual.array().pow(2);
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

double normaliseAngle(const double angle) {
	return atan2(sin(angle), cos(angle));
}
