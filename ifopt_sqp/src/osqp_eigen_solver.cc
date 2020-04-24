#include <ifopt/osqp_eigen_solver.h>

#include <OsqpEigen/OsqpEigen.h>

namespace osqp_eigen
{
bool OSQPEigenSolver::Init(ifopt::Problem& nlp)
{
  nlp_ = &nlp;
  status_ = true;
  solver_.clearSolver();

  num_vars_ = nlp_->GetNumberOfOptimizationVariables();
  num_cnts_ = nlp_->GetNumberOfConstraints();

  // set the initial data of the QP solver_
  solver_.data()->setNumberOfVariables(num_vars_);
  // OSQP does not have variable limits, so we set constraints on them
  solver_.data()->setNumberOfConstraints(num_cnts_ + num_vars_);

  return status_;
}

void OSQPEigenSolver::Solve()
{
  assert(nlp_->GetNumberOfConstraints() == num_cnts_);
  assert(nlp_->GetNumberOfOptimizationVariables() == num_vars_);

  solver_.clearSolver();

  ////////////////////////////////////////////////////////
  // Set the Hessian (empty for now)
  ////////////////////////////////////////////////////////
  solver_.data()->clearHessianMatrix();
  Eigen::SparseMatrix<double> hessian;
  hessian.resize(num_vars_, num_vars_);
  status_ &= solver_.data()->setHessianMatrix(hessian);

  ////////////////////////////////////////////////////////
  // Set the gradient
  ////////////////////////////////////////////////////////
  Eigen::VectorXd gradient(num_vars_);
  ifopt::ConstraintSet::Jacobian cost_jac = nlp_->GetJacobianOfCosts();
  gradient << cost_jac.toDense().transpose();
  status_ &= solver_.data()->setGradient(gradient);

  ////////////////////////////////////////////////////////
  // Linearize Constraints
  ////////////////////////////////////////////////////////
  Eigen::SparseMatrix<double> jac = nlp_->GetJacobianOfConstraints();

  // Create triplet list of nonzero constraints
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(jac.nonZeros());

  // Iterate over nonzero elements of jac and add them to triplet list
  for (int k = 0; k < jac.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it; ++it)
    {
      tripletList.push_back(T(it.row(), it.col(), it.value()));
    }
  }
  // Add a diagonal matrix for the variable limits below the actual constraints
  for (Eigen::Index i = 0; i < num_vars_; i++)
    tripletList.push_back(T(i + jac.rows(), i, 1));

  // Insert the triplet list into the sparse matrix
  Eigen::SparseMatrix<double> linearMatrix(num_cnts_ + num_vars_, num_vars_);
  linearMatrix.reserve(jac.nonZeros() + num_vars_);
  linearMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  // Set linear constraints
  solver_.data()->clearLinearConstraintsMatrix();
  status_ &= solver_.data()->setLinearConstraintsMatrix(linearMatrix);

  ////////////////////////////////////////////////////////
  // Set the bounds of the constraints
  ////////////////////////////////////////////////////////
  Eigen::VectorXd cnt_bound_lower(num_cnts_);
  Eigen::VectorXd cnt_bound_upper(num_cnts_);

  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = nlp_->GetBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_cnts_; i++)
  {
    cnt_bound_lower[i] = cnt_bounds[i].lower_;
    cnt_bound_upper[i] = cnt_bounds[i].upper_;
  }

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = nlp_->GetVariableValues();
  Eigen::VectorXd cnt_initial_value = nlp_->EvaluateConstraints(x_initial.data());

  // Our error is now represented as dy(x0)/dx * x + (y(x0) - dy(xo)/dx * x0)
  // This accounts for moving (error - dy/dx*x) term to other side of equation
  Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - (cnt_initial_value - jac * x_initial);
  Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - (cnt_initial_value - jac * x_initial);

  // Create full bounds vector
  Eigen::VectorXd full_bounds_lower(num_cnts_ + num_vars_);
  Eigen::VectorXd full_bounds_upper(num_cnts_ + num_vars_);

  // Insert linearized constraint bounds and check range - It might be faster to insert the whole vector and use eigen
  // to check the max/min values and only loop over the ones that need resetting
  for (Eigen::Index i = 0; i < num_cnts_; i++)
  {
    full_bounds_lower[i] = fmax(linearized_cnt_lower[i], -OSQP_INFTY);
    full_bounds_upper[i] = fmin(linearized_cnt_lower[i], OSQP_INFTY);
  }

  // Set the limits on the variables
  std::vector<ifopt::Bounds> var_bounds = nlp_->GetBoundsOnOptimizationVariables();
  for (Eigen::Index i = num_cnts_; i < (num_cnts_ + num_vars_); i++)
  {
    full_bounds_lower[i] = fmax(var_bounds[i].lower_, -OSQP_INFTY);
    full_bounds_upper[i] = fmin(var_bounds[i].upper_, OSQP_INFTY);
  }

  // Send the full bounds vector to OSQP
  status_ &= solver_.data()->setLowerBound(full_bounds_lower);
  status_ &= solver_.data()->setUpperBound(full_bounds_upper);

  ////////////////////////////////////////////////////////
  // Instantiate the solver
  ////////////////////////////////////////////////////////
  status_ &= solver_.initSolver();

  ////////////////////////////////////////////////////////
  // Solve and save the solution
  ////////////////////////////////////////////////////////
  status_ &= solver_.solve();
  results_ = solver_.getSolution();
}

}  // namespace osqp_eigen
