#pragma once
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;  // Not polite, but removing this changes the RMSE (!)

/*
 * Returns the RMSE, component by component, of the given estimations against the given ground
 * truth. The two given vectors must have the same size and be non-empty, otherwise the method
 * prints an error message to console and returns a vector with all zeroes.
 */
VectorXd calculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

/*
 * Returns the given angle, assumed in radians, normalised in the [-pi, pi] range.
 */
double normaliseAngle(const double angle);
