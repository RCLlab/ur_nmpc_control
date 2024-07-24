// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_ur_5.h"

#define NX     UR_5_NX
#define NZ     UR_5_NZ
#define NU     UR_5_NU
#define NP     UR_5_NP
#define NBX    UR_5_NBX
#define NBX0   UR_5_NBX0
#define NBU    UR_5_NBU
#define NSBX   UR_5_NSBX
#define NSBU   UR_5_NSBU
#define NSH    UR_5_NSH
#define NSG    UR_5_NSG
#define NSPHI  UR_5_NSPHI
#define NSHN   UR_5_NSHN
#define NSGN   UR_5_NSGN
#define NSPHIN UR_5_NSPHIN
#define NSBXN  UR_5_NSBXN
#define NS     UR_5_NS
#define NSN    UR_5_NSN
#define NG     UR_5_NG
#define NBXN   UR_5_NBXN
#define NGN    UR_5_NGN
#define NY0    UR_5_NY0
#define NY     UR_5_NY
#define NYN    UR_5_NYN
#define NH     UR_5_NH
#define NPHI   UR_5_NPHI
#define NHN    UR_5_NHN
#define NPHIN  UR_5_NPHIN
#define NR     UR_5_NR

class my_NMPC_solver {
private:
  int num_steps;
  int number_of_current_steps;
  // global data
  ur_5_solver_capsule * acados_ocp_capsule;
public:
  my_NMPC_solver(int n, int number_of_current_steps_specified) {
    number_of_current_steps = number_of_current_steps_specified;
    num_steps = n; // set number of real-time iterations
    ur_5_solver_capsule * my_acados_ocp_capsule = ur_5_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    
    int N = number_of_current_steps; // set variable step

    double stemps_array[number_of_current_steps]={0.0}; // create array of maximum length
    for (int i=0;i<number_of_current_steps;i++) stemps_array[i] = 0.2500;

    printf("Trying to create solver\n");
    double* new_time_steps = stemps_array;
    int status = ur_5_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
    if (status){
        printf("ur_5_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

  }
  int solve_my_mpc(double current_joint_position[6], double current_human_position[56], double current_joint_goal[6], double cgoal[3], double results[16], double eta) {
    int status = -1;
    int N = number_of_current_steps;

    printf("\n 1 STC  *********N = %i*********",N);
    ocp_nlp_config *nlp_config = ur_5_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = ur_5_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = ur_5_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = ur_5_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = ur_5_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = ur_5_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];   // indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    for (int i=0;i<NBX0;i++) idxbx0[i] = i;

    int idxbx[NBX];   // indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    for (int i=0;i<NBX;i++) idxbx[i] = i;

    int idxbu[NBU];
    for (int i=0;i<NBU;i++) idxbu[i] = i;
    
    int idxsh[NSH];
    for (int i=0;i<NSH;i++) idxsh[i]=i; 
    
    double lh[NH];
    double uh[NH];
    for (int i=0;i<NH;i++) lh[i] = -10e6;
    for (int i=0;i<NH;i++) uh[i] = 0.0;

    printf("\n N = %i NSH = %i NSHN = %i NH = %i\n", N, NSH, NSHN, NH);

    double lbx0[NBX0] = {0.0};
    double ubx0[NBX0] = {0.0};
    for (int i=0;i<6;i++)  {
        lbx0[i] = current_joint_position[i];
        ubx0[i] = current_joint_position[i];
    }
    
    // Set x0
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    double pi = 3.1415926;
    double lbx[6] = {-2*pi, -pi, -2*pi, -2*pi, -2*pi, -2*pi};
    double ubx[6] = { 2*pi,  0,   2*pi,  2*pi,  2*pi,  2*pi};

    double lbu[6] = {-1.2, -1.2, -1.2, -1.2, -1.2, -1.2};
    double ubu[6] = { 1.2,  1.2,  1.2,  1.2,  1.2,  1.2};

    double Vx[NY*NX] = {0.0};
    for (int i=0;i<NX;i++) Vx[NY*i+i] = 1.0;

    double Vx_e[NYN*NX] = {0.0};
    for (int i=0;i<NX;i++) Vx[NYN*i+i] = 1.0;

    double Vu[NY*NU] = {0.0};
    for (int i=0;i<NU;i++) Vu[NY*i+i] = 1.0;

    double W[(NU+NX)*(NU+NX)]= {0.0};
    double WN[(NX)*(NX)] = {0.0};

    double P = 68.4428;
    double sigma = 3.0;
    double Tweight = P*sigma*2;
    double weights_diag[NX] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; 
    for (int ii = 0; ii < (NX); ii++) W[ii+ii*(NU+NX)] = weights_diag[ii];  //position weights
    for (int ii = NU; ii < (NU+NX); ii++) W[ii+ii*(NU+NX)] = 1;  // velocity weights
    for (int ii = 0; ii < (NX); ii++) WN[ii+ii*(NX)] = Tweight;

    double zl[NSH] = {0.0};
    double zu[NSH] = {0.0};

    double Zl[NSH] = {0.0};
    double Zu[NSH] = {0.0}; 

    Zu[0] = 500.0; // Stick constraint

    double y_ref[NY];
    for (int i=0;i<6;i++) {
        y_ref[i] = current_joint_goal[i];
        y_ref[i+6] = 0.0;
    }
    double y_ref_N[NYN];
    for (int i=0;i<6;i++) y_ref_N[i] = current_joint_goal[i];

    double lh_e[1];
    lh_e[0] = -eta;
    double uh_e[1];
    uh_e[0] = 0;
    // Update constraints
    for (int ii = 0; ii < N; ii++){
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubu", ubu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "ubx", ubx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", uh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "idxsh", idxsh);   
    }

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "uh", uh_e);

    // Set x0
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", y_ref_N);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", y_ref_N);

    for (int ii=0;ii<N;ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", W);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);

    // Set goal
    for (int ii=0;ii<N;ii++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "zu", zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", y_ref);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", y_ref_N);

    // initialization for state values
    double x_init[NX*(N+1)]={0.0};
    for (int i=0; i<=N;i++) {
        for (int j=0;j<6;j++) {
            x_init[i*NX+j]=current_joint_position[j];
        }
    }
    // initial value for control input
    double u0[NU]={0.0};
    // set parameters
    double p[NP] = {0.0};
    for (int i = 0; i < 56; i++) p[i] = current_human_position[i];
    p[56] = cgoal[0]; p[57] = cgoal[1]; p[58] = cgoal[2];
    p[59] = current_joint_goal[0];
    p[60] = current_joint_goal[1];
    p[61] = current_joint_goal[2];
    p[62] = current_joint_goal[3];
    p[63] = current_joint_goal[4];
    p[64] = current_joint_goal[5];
    p[65] = eta;
    for (int ii = 0; ii < N; ii++) ur_5_acados_update_params(acados_ocp_capsule, ii, p, NP);
    ur_5_acados_update_params(acados_ocp_capsule, N, p, NP);
    
    // prepare evaluation
    int NTIMINGS = num_steps; // set number of real-time iterations
    double exec_time = 0.0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];

    // solve ocp in loop
    int rti_phase = 0;
    for (int ii = 0; ii < NTIMINGS; ii++){
        // initialize solution
        for (int i = 0; i <= nlp_dims->N; i++){
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init); 
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", y_ref_N);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = ur_5_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        exec_time = exec_time + elapsed_time;
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
    
    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);
    
    //***************************************
    results[15] = 0.0; // NaN marker
    for (int i=0;i<12;i++) {
        if (utraj[i]!=utraj[i]) results[17] = 0.0;
        else results[i] = utraj[i];
    }
    results[12] = sqp_iter; results[13] = exec_time*1000; results[14] = kkt_norm_inf;
    //***************************************
    if (status == ACADOS_SUCCESS)printf("ur_5_acados_solve(): SUCCESS!\n");
    else printf("ur_5_acados_solve() failed with status %d.\n", status);

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    printf(" SQP iterations %2d  minimum time for %d solve %f [ms]  KKT %e \n",
           sqp_iter, NTIMINGS, exec_time*1000, kkt_norm_inf);

    return status;
  } 

  int reset_solver(){
    int status = -1;
    // free solver
    status = ur_5_acados_free(acados_ocp_capsule);
    if (status) {
        printf("ur_5_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ur_5_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("ur_5_acados_free_capsule() returned status %d. \n", status);
    }
    return status;
  }


};
