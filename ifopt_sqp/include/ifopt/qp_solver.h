#ifndef IFOPT_INCLUDE_QP_SOLVER_H_
#define IFOPT_INCLUDE_QP_SOLVER_H_

#include <ifopt/problem.h>

namespace ifopt
{
class QPSolver
{
public:
  using Ptr = std::shared_ptr<QPSolver>;
  using ConstPtr = std::shared_ptr<const QPSolver>;

  QPSolver() = default;

  virtual bool Init(ifopt::Problem& nlp) = 0;

  virtual void Solve() = 0;

  virtual Eigen::VectorXd getResults() = 0;

  int getStatus() { return status_; };

protected:
  int status_;
};

}  // namespace ifopt

#endif
