#include <ifopt/cppoptlib_adapter.h>
#include <ifopt/constraint_set.h>

namespace cppoptlib
{
CppOptLibAdapter::CppOptLibAdapter(ifopt::Problem& nlp, bool finite_diff)
  : Superclass(nlp.GetNumberOfOptimizationVariables()), finite_diff_(std::move(finite_diff))
{
  nlp_ = &nlp;

  // Somehow set size?
}

double CppOptLibAdapter::value(const TVector& beta)
{
  Eigen::VectorXd cnt_evals = nlp_->EvaluateConstraints(beta.data());
  double cost_evals = nlp_->EvaluateCostFunction(beta.data());

  // TODO: Handle constraints
  double value = 100  * cnt_evals.sum() + cost_evals;

  return value;
}

void CppOptLibAdapter::gradient(const TVector& beta, TVector& grad)
{
  // Get constraint jacobian
    nlp_->SetVariables(beta.data());
  ifopt::ConstraintSet::Jacobian cnt_jacobian = nlp_->GetJacobianOfConstraints();

  // Get cost gradient
  ifopt::ConstraintSet::Jacobian cost_jacobian = nlp_->GetJacobianOfCosts();

  // compress into one gradient
//  assert(cnt_jacobian.cols() == cost_jacobian.cols());
  // TODO: Handle cases with no constraints or no costs

  Eigen::MatrixXd dense(cnt_jacobian.rows() + cost_jacobian.rows(), cnt_jacobian.cols());
  dense << cnt_jacobian.toDense(), cost_jacobian.toDense();
  Eigen::VectorXd result = dense.colwise().sum().transpose();
  grad = result;
}

}  // namespace cppoptlib
