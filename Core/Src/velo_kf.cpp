/*
 * velo_kf.cpp
 *
 *  Created on: Oct 5, 2026
 *      Author: Madelyn
 */

#include "velo_kf.h"
#include "Eigen/Core"

constexpr int N_STATES = 2;
constexpr int N_MEASUREMENTS = 2;

// States are x = [v_long, a_bias]
// TODO: Tune filter!!

Eigen::Matrix<float, N_STATES, N_STATES> Q {
	{0, 0},
	{0, 0}
};

Eigen::Matrix<float, N_STATES, N_STATES> R {
	{0, 0},
	{0, 0}
};

// Acceleration integrates into velocity, state update doesn't inform bias
Eigen::Matrix<float, N_STATES, N_STATES> A {
	{0, -1},
	{0, 0}
};

Eigen::Matrix<float, N_STATES, N_MEASUREMENTS> B {
	{1},
	{0}
};

Eigen::Matrix<float, N_STATES, N_STATES> H {
	{0, 0},
	{0, 0}
};

struct velo_kf_state {


	Eigen::Vector<float, N_STATES> x;
	Eigen::Matrix<float, N_STATES, N_STATES> P;

	// Inputs are u = [a_x]
	void predict(Eigen::Vector<float, 1> u, float dt) {

	}
};

static velo_kf_state s;
