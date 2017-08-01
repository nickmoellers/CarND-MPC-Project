#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.1;

const int delay_time = (int) dt/.1;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
  	double steer_value, double throttle_value);
};

// Evaluate a polynomial.
CppAD::AD<double> polyevalCppAd(Eigen::VectorXd coeffs, CppAD::AD<double> x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif /* MPC_H */
