#include <ifopt/cppoptlib_solver.h>
#include <ifopt/cppoptlib_adapter.h>

#include <cppoptlib/solver/lbfgsbsolver.h>
#include <cppoptlib/solver/gradientdescentsolver.h>
namespace ifopt
{
CppOptLibSolver::CppOptLibSolver()
{
  // No setup
}

void CppOptLibSolver::Solve(Problem& nlp)
{
  status_ = false;

  // Convert IFOPT to CppOptLib
  cppoptlib::CppOptLibAdapter adapter(nlp, finite_diff_);

  // Set Bounds
  std::vector<ifopt::Bounds> bounds = nlp.GetBoundsOnOptimizationVariables();
  cppoptlib::CppOptLibAdapter::TVector l_bound(bounds.size());
  cppoptlib::CppOptLibAdapter::TVector u_bound(bounds.size());
  for (std::size_t i = 0; i < bounds.size(); i++)
  {
    l_bound[static_cast<Eigen::Index>(i)] = bounds[i].lower_;
    u_bound[static_cast<Eigen::Index>(i)] = bounds[i].upper_;
  }
//  adapter.setBoxConstraint(l_bound, u_bound);


  // Set variables
  Eigen::VectorXd x = nlp.GetVariableValues();

  // Check gradient
    if(!adapter.checkGradient(x))
        std::cerr << "NLP Failed gradient check" << std::endl;

  // Solve
  cppoptlib::GradientDescentSolver<cppoptlib::CppOptLibAdapter> solver;
  solver.minimize(adapter, x);

  // Extract results
  nlp.SetVariables(x.data());

  status_ = true;
}

void CppOptLibSolver::SetOption(const std::string& name, const std::string& value)
{
  assert(name == "jacobian_approximation");
  if (value == "finite_difference-values")
    finite_diff_ = true;
  else
  {
    finite_diff_ = false;
    assert(value == "exact");
  }
}

void CppOptLibSolver::SetOption(const std::string& /*name*/, int /*value*/)
{
  std::cerr << "No CppOptLibSolver options settable using this method" << std::endl;
}

void CppOptLibSolver::SetOption(const std::string& /*name*/, double /*value*/)
{
  std::cerr << "No CppOptLibSolver options settable using this method" << std::endl;
}

double CppOptLibSolver::GetTotalWallclockTime()
{
  // TODO
}

int CppOptLibSolver::GetReturnStatus()
{
    return status_;
}

} /* namespace ifopt */
