#ifndef IFOPT_INCLUDE_CPPOPTLIB_ADAPTER_H_
#define IFOPT_INCLUDE_CPPOPTLIB_ADAPTER_H_

#include <cppoptlib/boundedproblem.h>

#include <ifopt/problem.h>

namespace cppoptlib
{

// This wraps the ifopt problem into a cppoptlib problem
class CppOptLibAdapter : public BoundedProblem<double>
{
public:
  using Superclass = BoundedProblem<double>;
  using typename Superclass::TVector;
  using TMatrix = typename Superclass::THessian;
public:
  CppOptLibAdapter(ifopt::Problem& nlp, bool finite_diff);

  double value(const TVector& beta);

  void gradient(const TVector& beta, TVector& grad);

private:
  ifopt::Problem* nlp_;
  bool finite_diff_;
};

}  // namespace cppoptlib

#endif
