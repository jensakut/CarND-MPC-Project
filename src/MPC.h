#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct result {
    vector<double> x;
    vector<double> y;
	vector<double> a;
    vector<double> delta;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  double delta_hist = 0;
  double a_hist = 0;
  int latency = 0;

};

#endif /* MPC_H */
