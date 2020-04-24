#ifndef IFOPT_INCLUDE_OSQP_EIGEN_ADAPTER_H_
#define IFOPT_INCLUDE_OSQP_EIGEN_ADAPTER_H_

#include <ifopt/problem.h>

namespace osqp_eigen
{

// This wraps the ifopt problem into a QP problem
class OSQPEigenAdapter
{
public:
  OSQPEigenAdapter(ifopt::Problem& nlp);

//  double value(const TVector& beta);

//  void gradient(const TVector& beta, TVector& grad);

private:
  ifopt::Problem* nlp_;
};

}  // namespace cppoptlib

#endif
