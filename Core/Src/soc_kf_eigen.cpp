/*
 * soc_kf_eigen.cpp
 *
 *  Created on: Sep 22, 2026
 *      Author: Madelyn
 */

// Discharge positive

#include "soc_kf_eigen.h"
#include "Eigen/Core"

// 1d soc lookups, doesn't key on temperature

float C1_lookup(float soc) {
	return 0;
}

float R1_lookup(float soc) {
	return 0;
}

float V0_lookup(float soc) {
	return 0;
}

float dV0_dSOC(float soc) {
	return 0;
}

float R0_lookup(float soc) {
	return 0;
}


// Discretization sample time, assumed constant
// should be measured?

float T_s = 0;

// Battery capacity

float Q_Ah = 4.9901f;

struct soc_kf_state {
	Eigen::Vector<float, 2> x;
	Eigen::Matrix<float, 2, 2> P;
	Eigen::Matrix<float, 2, 1> K;
};

// Uncertainty per second
Eigen::Matrix<float, 2, 2> Q {
	{1.1e-9f, 0},
	{0, 1.1e-5f}
};

Eigen::Matrix<float, 1, 1> R {
	1e-3f
};

// Nonlinear state transition

Eigen::Vector<float, 2> f(Eigen::Vector<float, 2> x, float current) {
	float soc = x[0];
	float V1 = x[1];

	// ZOH discrete time integration constant
	auto a = std::exp(-T_s/(R1_lookup(soc) * C1_lookup(soc)));

	return Eigen::Vector<float, 2> {
		soc - (current*T_s)/(3600.0f * Q_Ah),
		V1 * a + current * R1_lookup(soc) * (1 - a)
	};
}

// Nonlinear observation function

Eigen::Vector<float, 1> h(Eigen::Vector<float, 2> state, float current) {
	float soc = state[0];
	float V1 = state[1];

	return Eigen::Vector<float, 1> {
		V0_lookup(soc) - current * R0_lookup(soc) - V1
	};
}


// Linearized state transition

Eigen::Matrix<float, 2, 2> F_d(Eigen::Vector<float, 2> x) {
	float soc = x[0];

	return Eigen::Matrix<float, 2, 2> {
		{1, 0},
		{0, std::exp(-T_s/(R1_lookup(soc) * C1_lookup(soc)))}
	};
}

// Linearized observation function

Eigen::Matrix<float, 1, 2> H_d(Eigen::Vector<float, 2> x) {
	float soc = x[0];

	return Eigen::Matrix<float, 1, 2> {
		{dV0_dSOC(soc), -1}
	};
}

// predict-correct loop

void predict(struct soc_kf_state* state, float current) {
	auto F_d_ = F_d(state->x);

	// state transition
	state->x = f(state->x, current);

	// extrapolate uncertainty
	state->P = F_d_ * state->P * F_d_.transpose() + Q * T_s;
}

void correct(struct soc_kf_state* state, float current, float Vt) {
	auto H_d_ = H_d(state->x);

	// compute Kalman gain
	// we use scalar division to invert here since the matrix is 1x1
	state->K = (state->P * H_d_.transpose())/(H_d_ * state->P * H_d_.transpose() + R).value();

	// update estimate
	state->x = state->x + state->K * (Vt - h(state->x, current).value());

	// update uncertainty
	state->P = (Eigen::Matrix<float, 2, 2>::Identity() - state->K * H_d_) * state->P * (Eigen::Matrix<float, 2, 2>::Identity() - state->K * H_d_).transpose() + state->K * R * state->K.transpose();
}

float get_soc(struct soc_kf_state* state) {
	float soc = state->x[0];
	return soc;
}

float get_V1(struct soc_kf_state* state) {
	float V1 = state->x[1];

	return V1;
}
