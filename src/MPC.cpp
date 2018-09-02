#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const size_t x_start 		= 0;
const size_t y_start		= x_start + N;
const size_t psi_start		= y_start + N;
const size_t v_start		= psi_start + N;
const size_t cte_start		= v_start + N;
const size_t epsi_start		= cte_start + N;
const size_t delta_start	= epsi_start + N;
const size_t a_start		= delta_start + N - 1;

class FG_eval 
{
	public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	
	void operator()(ADvector& fg, const ADvector& vars) 
	{
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, 
		// `vars` is a vector of variable values (state & actuators)
		
		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;
		
		// The part of the cost based on the reference state.
		// In each iteration through the loop, we sum three components 
		// to reach the aggregate cost: our cross-track error, 
		// our heading error, and our velocity error.
		for (unsigned int t = 0; t < N; t++) 
		{
			fg[0] += 1000*CppAD::pow(vars[cte_start + t], 2);
			fg[0] += 1000*CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += CppAD::pow(vars[v_start + t]-100, 2);
		}
		
		// Minimize the use of actuators.
		// For example, if we're making a turn, we'd like the turn to 
		// be smooth, not sharp. Additionally, the vehicle velocity should 
		// not change too radically.
		for (unsigned int t = 0; t < N - 1; t++) 
		{
			fg[0] += 50*CppAD::pow(vars[delta_start + t], 2);
			fg[0] += 50*CppAD::pow(vars[a_start + t], 2);
		}
		
		// Minimize the value gap between sequential actuations.
		// The goal is to make control decisions more consistent, or smoother, that is,
		// the next control input should be similar to the current one.
		for (unsigned int t = 0; t < N - 2; t++) 
		{
			fg[0] += 250000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += 5000*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Initialize constraints	
		// We initialize the model to the initial state. 
		// Since fg[0] is reserved for the cost value, 
		// so the other indices are bumped up by 1.
		fg[1 + x_start] 	= vars[x_start];
		fg[1 + y_start] 	= vars[y_start];
		fg[1 + psi_start] 	= vars[psi_start];
		fg[1 + v_start] 	= vars[v_start];
		fg[1 + cte_start] 	= vars[cte_start];
		fg[1 + epsi_start] 	= vars[epsi_start];
		
		for (unsigned int t = 1; t < N ; t++) 
		{
			// State at time t
			AD<double> x0 		= vars[x_start 		+ t - 1];
			AD<double> y0 		= vars[y_start 		+ t - 1];
			AD<double> psi0 	= vars[psi_start 	+ t - 1];
			AD<double> v0 		= vars[v_start 		+ t - 1];
			AD<double> cte0 	= vars[cte_start 	+ t - 1];
			AD<double> epsi0 	= vars[epsi_start 	+ t - 1];						
			
			// State at time t+1
			AD<double> x1 		= vars[x_start 		+ t];
			AD<double> y1 		= vars[y_start 		+ t];
			AD<double> psi1 	= vars[psi_start 	+ t];
			AD<double> v1 		= vars[v_start 		+ t];
			AD<double> cte1 	= vars[cte_start 	+ t];
			AD<double> epsi1 	= vars[epsi_start 	+ t];

			// Only consider the actuation at time t.
			AD<double> delta0 	= vars[delta_start 	+ t - 1];
			AD<double> a0	 	= vars[a_start 	+ t - 1];
			
			AD<double> f0 = coeffs[0] + 
							coeffs[1] * x0 +
							coeffs[2] * CppAD::pow(x0, 2) +
							coeffs[3] * CppAD::pow(x0, 3);
							
			AD<double> psides0 = 	CppAD::atan(coeffs[1]) +
									2 * coeffs[2] * x0 + 
									3 * coeffs[3] * CppAD::pow(x0, 2);

			// Recall the equations for the model:
			// x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
			// y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
			// psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
			// v_[t] = v[t-1] + a[t-1] * dt
			// cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
			// epsi[t] = psi[t-1] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt					
			
			fg[1 + x_start + t] 	= x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] 	= y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			// modified psi update
			fg[1 + psi_start + t] 	= psi1 - (psi0 - v0/Lf * delta0 * dt);
			fg[1 + v_start + t] 	= v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] 	= cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
			fg[1 + epsi_start + t] 	= epsi1 - (psi0 - psides0 + v0/Lf * delta0 * dt);						
									
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;
	
	// Collect state values
	const double x 		= state[0];
	const double y 		= state[1];
	const double psi 	= state[2];
	const double v 		= state[3];
	const double cte	= state[4];
	const double epsi	= state[5];

	// TODO: Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	//
	// 4 * 10 + 2 * 9
	size_t n_vars = 6 * N + 2 * (N-1);
	/////////////////////////////////////////////////////////////////////////
	
	// TODO: Set the number of constraints
	size_t n_constraints = 6 * N;
	/////////////////////////////////////////////////////////////////////////

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	/////////////////////////////////////////////////////////////////////////
	Dvector vars(n_vars);
	for (i = 0; i < n_vars; i++) 
	{
		vars[i] = 0;
	}
	/////////////////////////////////////////////////////////////////////////
	
	// Set lower and upper limits for variables.
	/////////////////////////////////////////////////////////////////////////
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);	

	for (i = 0; i < delta_start; i++)  // All non-actuators and non-delta
	{
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}
	
	for (i = delta_start; i < a_start; i++) // Limits of delta 
	{
		vars_lowerbound[i] = -0.436332 * Lf; // -25 degrees (in radian)
		vars_upperbound[i] = 0.436332 * Lf;  // 25 degrees (in radian)
	}
	
	for (i = a_start; i < n_vars; i++)  // Actuator limits
	{
		vars_lowerbound[i] = -1.0; 
		vars_upperbound[i] = 1.0;  
	}
	/////////////////////////////////////////////////////////////////////////
	
	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	/////////////////////////////////////////////////////////////////////////
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (i = 0; i < n_constraints; i++) 
	{
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	
	constraints_lowerbound[x_start] 	= x;
	constraints_lowerbound[y_start] 	= y;
	constraints_lowerbound[psi_start] 	= psi;
	constraints_lowerbound[v_start] 	= v;
	constraints_lowerbound[cte_start] 	= cte;
	constraints_lowerbound[epsi_start] 	= epsi;

	constraints_upperbound[x_start] 	= x;
	constraints_upperbound[y_start] 	= y;
	constraints_upperbound[psi_start] 	= psi;
	constraints_upperbound[v_start] 	= v;
	constraints_upperbound[cte_start] 	= cte;
	constraints_upperbound[epsi_start] 	= epsi;
	/////////////////////////////////////////////////////////////////////////
	
	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	//auto cost = solution.obj_value;
	//std::cout << "Cost " << cost << std::endl;

	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.
	vector <double> result;
	result.push_back(solution.x[delta_start]);
	result.push_back(solution.x[a_start]);
	
	for(i = 0; i < N-2; ++i)
	{
		result.push_back(solution.x[x_start + i + 1]);
		result.push_back(solution.x[y_start + i + 1]);	
	}
	return result;
}
