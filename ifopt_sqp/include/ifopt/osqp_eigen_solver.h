#ifndef IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace osqp_eigen
{
/**
 * @brief Provides an interface to OSQP via the osqp_eigen library
 *
 * For more details see here:
 * https://github.com/robotology/osqp-eigen
 */
class OSQPEigenSolver : public ifopt::QPSolver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver() = default;

  /**
   * @brief Initializes the solver. This should be called any time costs, constraints, or variables are added or
   * modified
   * @param nlp The IFOPT problem to be approximated and solved. Must remain in scope until Solve() is called
   * @return true if succeeded
   */
  bool Init(ifopt::Problem& nlp) override;

  /** @brief Linearizes the problem about the current variable values and solves the QP */
  void Solve() override;

  /** @brief Get the results vector after Solve()  */
  Eigen::VectorXd getResults() override { return results_; };

  /** @brief Low level SQP solver. Use solver_.settings() to change settings */
  OsqpEigen::Solver solver_;

private:
  ifopt::Problem* nlp_;

  Eigen::Index num_vars_;
  Eigen::Index num_cnts_;
  Eigen::VectorXd results_;
};

}  // namespace osqp_eigen

#endif
