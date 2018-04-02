#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);

  // Returns a new predicted future state vector given current state, actuators to apply and duration
  Eigen::VectorXd globalKinematic(const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &actuators, double dt);
};

#endif /* MPC_H */
