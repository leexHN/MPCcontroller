#include "mex.h"
#include "car_kin_mpc.h"

void d_array2eigen(Eigen::MatrixXd & eigen, const double* array);
void d_eigen2array(const Eigen::MatrixXd & eigen, double* array);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	controller::CarKinMPC car_kin_mpc;

	controller::CarParm car_parm;
	car_parm.wheel_base = 3.0;
	car_parm.max_delta_f = DEG2RAD(30);
	car_parm.max_delta_f_rate = DEG2RAD(15);
	car_parm.max_v = 5;
	car_parm.min_v = -5;
	car_parm.max_acc = 1;
	car_parm.max_decel = -1;

	controller::MPCConfig mpc_config;
	mpc_config.np = 20;
	mpc_config.nc = 5;
	mpc_config.ts = 0.05;
	mpc_config.q = 10 * Eigen::MatrixXd::Identity(3, 3);
	mpc_config.r = Eigen::MatrixXd::Identity(2, 2);

	car_kin_mpc.Init(mpc_config, car_parm);

	// prhs
	double *p_ref, *p_state;
	Eigen::MatrixXd ref(mpc_config.np*3,1), state(5,1);

	p_ref = mxGetPr(prhs[0]);
	p_state = mxGetPr(prhs[1]);
	d_array2eigen(ref, p_ref);
	d_array2eigen(state, p_state);

	Eigen::MatrixXd result;
	car_kin_mpc.CurrentState(state);
	result = car_kin_mpc.ComputeControlCommand(ref);

	if (result.size() < 1)  mexErrMsgTxt("MPC failed");
	plhs[0] = mxCreateDoubleMatrix(result.rows(), result.cols(), mxREAL);
	
	double * p_result;
	p_result = mxGetPr(plhs[0]);
	d_eigen2array(result, p_result);
}

void d_array2eigen(Eigen::MatrixXd & eigen,const double* array) {
	for (size_t i = 0; i < eigen.size(); i++)
	{
		eigen(i) = array[i];
	}
}

void d_eigen2array(const Eigen::MatrixXd & eigen, double* array) {
	for (size_t i = 0; i < eigen.size(); i++)
	{
		array[i]=eigen(i);
	}
}