#include <ifopt/osqp_eigen_adapter.h>
#include <ifopt/osqp_eigen_solver.h>

#include <OsqpEigen/OsqpEigen.h>

namespace ifopt
{
OSQPEigenSolver::OSQPEigenSolver()
{
  // No setup
}

void OSQPEigenSolver::Solve(Problem& nlp)
{
  status_ = true;
  OsqpEigen::Solver solver;

  // settings
   solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  Eigen::Index num_vars = nlp.GetNumberOfOptimizationVariables();
  Eigen::Index num_cnts = nlp.GetNumberOfConstraints();

  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(num_vars);
  // OSQP does not have variable limits, so we set constraints on them
  solver.data()->setNumberOfConstraints(num_cnts + num_vars);

  ////////////////////////////////////////////////////////
  // Leave Hessian empty for now
  Eigen::SparseMatrix<double> hessian;
  hessian.resize(num_vars, num_vars);
  status_ &= solver.data()->setHessianMatrix(hessian);

  /////////////////////////////////////////////////
  // Set gradient
  Eigen::VectorXd gradient(num_vars);
  ifopt::ConstraintSet::Jacobian cost_jac = nlp.GetJacobianOfCosts();
  gradient << cost_jac.toDense().transpose();
  status_ &= solver.data()->setGradient(gradient);

  /////////////////////////////////////
  // Linearize constraints
  Eigen::SparseMatrix<double> jac = nlp.GetJacobianOfConstraints();

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
  for (Eigen::Index i = 0; i < num_vars; i++)
    tripletList.push_back(T(i + jac.rows(), i, 1));

  // Insert the triplet list into the sparse matrix
  Eigen::SparseMatrix<double> linearMatrix(num_cnts + num_vars, num_vars);
  linearMatrix.reserve(jac.nonZeros() + num_vars);
  linearMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  // Set linear constraints
  status_ &= solver.data()->setLinearConstraintsMatrix(linearMatrix);

  ///////////////////////////////////////
  // Set the bounds of the constraints
  Eigen::VectorXd cnt_bound_lower(num_cnts);
  Eigen::VectorXd cnt_bound_upper(num_cnts);

  // Convert constraint bounds to VectorXd
  std::vector<ifopt::Bounds> cnt_bounds = nlp.GetBoundsOnConstraints();
  for (Eigen::Index i = 0; i < num_cnts; i++)
  {
    cnt_bound_lower[i] = cnt_bounds[i].lower_;
    cnt_bound_upper[i] = cnt_bounds[i].upper_;
  }

  // Get values about which we will linearize
  Eigen::VectorXd x_initial = nlp.GetVariableValues();
  Eigen::VectorXd cnt_initial_value = nlp.EvaluateConstraints(x_initial.data());

  // Our error is now represented as dy(x0)/dx * x + (y(x0) - dy(xo)/dx * x0)
  // This accounts for moving (error - dy/dx*x) term to other side of equation
  Eigen::VectorXd linearized_cnt_lower = cnt_bound_lower - (cnt_initial_value - jac * x_initial);
  Eigen::VectorXd linearized_cnt_upper = cnt_bound_upper - (cnt_initial_value - jac * x_initial);

  // Create full bounds vector
  Eigen::VectorXd full_bounds_lower(num_cnts + num_vars);
  Eigen::VectorXd full_bounds_upper(num_cnts + num_vars);

  // Insert linearized constraint bounds and check range - It might be faster to insert the whole vector and use eigen
  // to check the max/min values and only loop over the ones that need resetting
  for (Eigen::Index i = 0; i < num_cnts; i++)
  {
    full_bounds_lower[i] = fmax(linearized_cnt_lower[i], -OSQP_INFTY);
    full_bounds_upper[i] = fmin(linearized_cnt_lower[i], OSQP_INFTY);
  }

  // Set the limits on the variables
  std::vector<ifopt::Bounds> var_bounds = nlp.GetBoundsOnOptimizationVariables();
  for (Eigen::Index i = num_cnts; i < (num_cnts + num_vars); i++)
  {
    full_bounds_lower[i] = fmax(var_bounds[i].lower_, -OSQP_INFTY);
    full_bounds_upper[i] = fmin(var_bounds[i].upper_, OSQP_INFTY);
  }

  // Send the full bounds vector to OSQP
  status_ &= solver.data()->setLowerBound(full_bounds_lower);
  status_ &= solver.data()->setUpperBound(full_bounds_upper);

  //////////////////////////////////////
  // instantiate the solver
  status_ &= solver.initSolver();


  //////////////////////////////
  // Solve
  status_ &=solver.solve();

  Eigen::VectorXd solution;
  solution = solver.getSolution();

  nlp.SetVariables(solution.data());
}

// void OSQPEigenSolver::SetOption(const std::string& name, const std::string& value)
//{
//  assert(name == "jacobian_approximation");
//  if (value == "finite_difference-values")
//    finite_diff_ = true;
//  else
//  {
//    finite_diff_ = false;
//    assert(value == "exact");
//  }
//}

// void OSQPEigenSolver::SetOption(const std::string& /*name*/, int /*value*/)
//{
//  std::cerr << "No OSQPEigenSolver options settable using this method" << std::endl;
//}

// void OSQPEigenSolver::SetOption(const std::string& /*name*/, double /*value*/)
//{
//  std::cerr << "No OSQPEigenSolver options settable using this method" << std::endl;
//}

// double OSQPEigenSolver::GetTotalWallclockTime()
//{
//  // TODO
//}

 int OSQPEigenSolver::GetReturnStatus()
{
    return status_;
}

} /* namespace ifopt */
