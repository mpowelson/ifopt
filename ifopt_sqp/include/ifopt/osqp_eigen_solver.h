#ifndef IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace osqp_eigen
{
class OSQPEigenSolver : public ifopt::QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver() = default;

  bool Init(ifopt::Problem& nlp) override;

  void Solve() override;

  Eigen::VectorXd getResults() override { return results_; };

private:
  ifopt::Problem* nlp_;
  OsqpEigen::Solver solver_;
  Eigen::Index num_vars_;
  Eigen::Index num_cnts_;
  Eigen::VectorXd results_;
};

}  // namespace osqp_eigen

#endif
