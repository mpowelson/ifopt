#include <ifopt/simple_sqp_solver.h>
#include <iostream>

namespace ifopt
{
SimpleSQPSolver::SimpleSQPSolver(QPSolver::Ptr qp_solver) : qp_solver_(qp_solver) {}

void SimpleSQPSolver::Solve(Problem& nlp)
{
  qp_solver_->Init(nlp);
  status_ = qp_solver_->getStatus();

  Eigen::VectorXd results_previous;
  for (int ind = 0; ind < 100; ind++)
  {
    ////////////////////////////////////////////////////////
    // Solve the QP
    ////////////////////////////////////////////////////////
    qp_solver_->Solve();
    Eigen::VectorXd results = qp_solver_->getResults();

    ////////////////////////////////////////////////////////
    // Evaluate the results
    ////////////////////////////////////////////////////////
    nlp.SetVariables(results.data());
    double objective = nlp.EvaluateCostFunction(results.data());

    ////////////////////////////////////////////////////////
    // Call callbacks
    ////////////////////////////////////////////////////////
    std::cout << results.transpose() << "  |  "
              << "Obj: " << objective << std::endl;

    ////////////////////////////////////////////////////////
    // Check termination criteria
    ////////////////////////////////////////////////////////
    // Solver failed
    if (!qp_solver_->getStatus())
    {
      status_ = false;
      break;
    }
    // Check convergence
    if (ind > 0 && results.isApprox(results_previous))
    {
      break;
    }
    results_previous = results;
  }
}

int SimpleSQPSolver::GetReturnStatus() { return status_; }
}  // namespace ifopt
