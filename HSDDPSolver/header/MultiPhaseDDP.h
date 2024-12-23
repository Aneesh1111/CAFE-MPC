#ifndef MULTIPHASEDDP_H
#define MULTIPHASEDDP_H

#include <vector>
#include <deque>
#include <memory> // smart pointer (since C++ 11)
#include <functional>
#include <utility> // std::pair, tuple
#include <tuple>
#include <lcm/lcm-cpp.hpp>
#include <chrono>

#include "SinglePhase.h"
#include "solver_intermtraj_lcmt.hpp"
#include "utilities.h"

using std::vector;
using std::deque;
using std::shared_ptr;
using std::function;
using namespace std::chrono;
using duration_ms = std::chrono::duration<float, std::chrono::milliseconds::period>;

template <typename T>
class MultiPhaseDDP
{
private:
    deque<shared_ptr<SinglePhaseBase<T>>> phases;

public:
    MultiPhaseDDP(): traj_lcm(getLcmUrl(255)) {} // default constructor

    void set_multiPhaseProblem(deque<shared_ptr<SinglePhaseBase<T>>> phases_in){
        phases = phases_in;
        n_phases = phases.size();
        actual_cost = 0;        
        max_pconstr = 0;
        max_pconstr_prev = 0;
        max_tconstr = 0;
        max_tconstr_prev = 0;
    }

    void set_initial_condition(DVec<T> x0_in) { x0 = x0_in; dx0.setZero(x0.size()); }

    void solve(HSDDP_OPTION& option, const float& max_cputime=1e100); // Default max_cputime 100 s    

    void set_dynamics_init_callback(function<void(DVec<T>)> dynamics_init_callback_);

public:

    void linear_rollout(T eps, HSDDP_OPTION& option);

    bool hybrid_rollout(T eps, HSDDP_OPTION& option);

    std::pair<bool, int> line_search(HSDDP_OPTION& option);

    void compute_cost(const HSDDP_OPTION& option);

    void LQ_approximation(HSDDP_OPTION& option);

    bool backward_sweep(T regularization);

    std::pair<bool, int> backward_sweep_regularized(T& regularization, HSDDP_OPTION&option);

    void impact_aware_step(DVec<T>&G, DMat<T>&H, const DMat<T>& Px);

    void update_nominal_trajectory();

    void run_before_forward_sweep();

    void update_AL_params(HSDDP_OPTION& option);

    void update_REB_params(HSDDP_OPTION& option);    

    T measure_dynamics_feasibility(int norm_id=2);   

    T get_actual_cost() {return actual_cost;}   

    T get_dyn_infeasibility() {return feas;} 

    T get_path_constraint_violation() {return ineq_feas_buffer.back();}

    T get_terminal_constraint_violation() {return eqn_feas_buffer.back();}

    void get_solver_info(std::vector<float>&, std::vector<float>&,
                         std::vector<float>&, std::vector<float>&);

    void get_solver_info(int& n_iters, int& n_ls_iters, int& n_reg_iters, float& solve_time){
        n_iters = iter_;
        n_ls_iters = ls_iter_total_;
        n_reg_iters = reg_iter_total_;
        solve_time = solve_time_;
    }
   
    void publish_trajectory(){
        traj_to_publish.x_tau.clear();
        traj_to_publish.u_tau.clear();
        for (auto& phase: phases)
        {
            phase->get_trajectory(traj_to_publish.x_tau, traj_to_publish.u_tau);
        }
        traj_to_publish.tau_sz = traj_to_publish.x_tau.size();
        traj_to_publish.x_sz = traj_to_publish.x_tau[0].size();
        traj_to_publish.u_sz = traj_to_publish.u_tau[0].size();
        traj_lcm.publish("intermediate_ddp_traj", &traj_to_publish);
        printf("publishing an intermediate trajectory \n");
    }

private:
    int n_phases;
    int iter_{0};
    int ls_iter_total_{0};
    int reg_iter_total_{0};
    float solve_time_{0};

    T actual_cost{0};
    T merit{0};
    T feas{0};
    // T exp_cost_change;
    T dV_1;         // expected cost change due to first order
    T dV_2;         // expected cost change due to second order

    T max_tconstr_prev;
    T max_pconstr_prev;
    T max_tconstr; // maximum terminal contraint violation of all time
    T max_pconstr; // maximum path constraint violation of all time
    T merit_rho;   // penalty parameter of the merit function

    DVec<T> x0;
    DVec<T> dx0;

    // buffered solver information
    std::vector<float> cost_buffer;    
    std::vector<float> dyn_feas_buffer;
    std::vector<float> eqn_feas_buffer;
    std::vector<float> ineq_feas_buffer; 

    // helpful variables
    lcm::LCM traj_lcm;
    solver_intermtraj_lcmt traj_to_publish;    

private:
    function<void(DVec<T>)> dynamics_init_callback;

};

#endif // MULTIPHASEDDP_H