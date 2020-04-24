#ifndef IFOPT_INCLUDE_SIMPLE_SQP_SOLVER_H_
#define IFOPT_INCLUDE_SIMPLE_SQP_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>
#include <ifopt/qp_solver.h>

namespace ifopt
{
/**
 * @brief A simple SQP Solver that uses the QPSolver passed in
 *
 * @ingroup Solvers
 */
class SimpleSQPSolver : public Solver
{
public:
  using Ptr = std::shared_ptr<SimpleSQPSolver>;
  using ConstPtr = std::shared_ptr<const SimpleSQPSolver>;

  SimpleSQPSolver(QPSolver::Ptr qp_solver);
  virtual ~SimpleSQPSolver() = default;

  void Solve(Problem& nlp) override;

  /** @brief  Get the return status for the optimization.*/
  int GetReturnStatus();

private:
  int status_;

  QPSolver::Ptr qp_solver_;
};

} /* namespace ifopt */

#endif
