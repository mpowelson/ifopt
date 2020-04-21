#ifndef IFOPT_SRC_IFOPT_IPOPT_INCLUDE_CPPOPTLIB_H
#define IFOPT_SRC_IFOPT_IPOPT_INCLUDE_CPPOPTLIB_H

#include <ifopt/problem.h>
#include <ifopt/solver.h>

namespace ifopt
{
/**
 * @brief An interface to cppoptlib, fully hiding its implementation.
 *
 * For more details, see:
 * https://github.com/PatWie/CppNumericalSolvers
 *
 * @ingroup Solvers
 */
class CppOptLibSolver : public Solver
{
public:
  using Ptr = std::shared_ptr<CppOptLibSolver>;
  using ConstPtr = std::shared_ptr<const CppOptLibSolver>;

  CppOptLibSolver();
  virtual ~CppOptLibSolver() = default;

  /** @brief Converts the ifopt::Problem into an cppoptlib::CppOptLibAdapter and solves the NLP.
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
