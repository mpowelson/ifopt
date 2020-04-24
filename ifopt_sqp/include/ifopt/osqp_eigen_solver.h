#ifndef IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_
#define IFOPT_INCLUDE_OSQP_EIGEN_SOLVER_H_

#include <ifopt/problem.h>
#include <ifopt/solver.h>

namespace ifopt
{
/**
 * @brief An interface to OSQPEigen, fully hiding its implementation.
 *
 * @ingroup Solvers
 */
class OSQPEigenSolver : public Solver
{
public:
  using Ptr = std::shared_ptr<OSQPEigenSolver>;
  using ConstPtr = std::shared_ptr<const OSQPEigenSolver>;

  OSQPEigenSolver();
  virtual ~OSQPEigenSolver() = default;

  /** @brief Converts the ifopt::Problem into an OSQPEigen::OSQPEigenAdapter and solves the NLP.
   * @param [in/out]  nlp  The specific IFOPT problem.
   */
  void Solve(Problem& nlp) override;

  void SetOption(const std::string& name, const std::string& value);
  void SetOption(const std::string& name, int value);
  void SetOption(const std::string& name, double value);

  /** @brief  Get the total wall clock time for the optimization, including function evaluations.
   */
  double GetTotalWallclockTime();

  /** @brief  Get the return status for the optimization.
   */
  int GetReturnStatus();

private:
  int status_;
  bool finite_diff_;
};

} /* namespace ifopt */

#endif
