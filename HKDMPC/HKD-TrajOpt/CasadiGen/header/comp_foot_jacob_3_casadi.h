/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int comp_foot_jacob_3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int comp_foot_jacob_3_alloc_mem(void);
extern "C" int comp_foot_jacob_3_init_mem(int mem);
extern "C" void comp_foot_jacob_3_free_mem(int mem);
extern "C" int comp_foot_jacob_3_checkout(void);
extern "C" void comp_foot_jacob_3_release(int mem);
extern "C" void comp_foot_jacob_3_incref(void);
extern "C" void comp_foot_jacob_3_decref(void);
extern "C" casadi_int comp_foot_jacob_3_n_out(void);
extern "C" casadi_int comp_foot_jacob_3_n_in(void);
extern "C" casadi_real comp_foot_jacob_3_default_in(casadi_int i);
extern "C" const char* comp_foot_jacob_3_name_in(casadi_int i);
extern "C" const char* comp_foot_jacob_3_name_out(casadi_int i);
extern "C" const casadi_int* comp_foot_jacob_3_sparsity_in(casadi_int i);
extern "C" const casadi_int* comp_foot_jacob_3_sparsity_out(casadi_int i);
extern "C" int comp_foot_jacob_3_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
