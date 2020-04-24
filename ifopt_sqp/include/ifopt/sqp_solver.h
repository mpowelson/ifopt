#ifndef IFOPT_INCLUDE_SQP_SOLVER_H_
#define IFOPT_INCLUDE_SQP_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <ifopt/qp_solver.h>

namespace ifopt
{
/**
 * @brief An interface to OSQPEigen, fully hiding its implementation.
 *
 * @ingroup Solvers
 */
class SQPSolver : public Solver
{
public:
  using Ptr = std::shared_ptr<SQPSolver>;
  using ConstPtr = std::shared_ptr<const SQPSolver>;

  SQPSolver(QPSolver::Ptr qp_solver);
  virtual ~SQPSolver() = default;

  /** @brief Converts the ifopt::Problem into an OSQPEigen::OSQPEigenAdapter and solves the NLP.
   * @param [in/out]  nlp  The specific IFOPT problem.
   */
  void Solve(Problem& nlp) override;

  /** @brief  Get the return status for the optimization.
   */
  int GetReturnStatus();

private:
  int status_;
  bool finite_diff_;

  QPSolver::Ptr qp_solver_;
};

} /* namespace ifopt */

#endif
