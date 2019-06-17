#include "car_kin_mpc.h"
#include <math.h>

#define name2str(name) (#name)

using Matrix = Eigen::MatrixXd;

controller::CarKinMPC::CarKinMPC()
{
	controller_info_ << "====Controller Info=======" << std::endl;
	controller_info_ << "name = car kinetic MPC Controller\n";
	solver_log_("solver.log");
	car_kin_log_("car_kin.log");
	mpcslover_(&solver_log_);
}

void controller::CarKinMPC::Init(MPCConfig mpc_config)
{
	mpc_config_ = mpc_config;
	matrix_u_ = Matrix::Zero(2, 1); // default input v=0,delta_f=0
	
	size_t nu = 2, nc = mpc_config.nc;
	Matrix lb(nu,1), ub(nu, 1), s_lb(nu, 1), s_ub(nu, 1);

	lb << max_decel_ ,-max_delta_f_rate_;
	ub << max_acc_, max_delta_f_rate_;
	s_lb << min_v_, -max_delta_f_;
	s_ub << max_v_, max_delta_f_;

	mpcslover_.Init(mpc_config.np, mpc_config.nc,
		mpc_config.ts, mpc_config.ts*lb, mpc_config.ts*ub, s_lb, s_ub);

	matrix_u_ = Matrix::Zero(nu, 1);
	matrix_del_u_ = Matrix::Zero(nu, 1);

	car_kin_log_ << "====MPC INFO=======" << std::endl;
	car_kin_log_ << name2str(mpc_config.np) << "=" << mpc_config.np << std::endl;
	car_kin_log_ << name2str(mpc_config.nc) << "=" << mpc_config.nc << std::endl;
	car_kin_log_ << name2str(mpc_config.ts) << "=" << mpc_config.ts << std::endl;
	car_kin_log_ << name2str(mpc_config.q) << "=" << mpc_config.q << std::endl;
	car_kin_log_ << name2str(mpc_config.r) << "=" << mpc_config.r << std::endl;

	car_kin_log_ << "====Constain=======" << std::endl;
	car_kin_log_ << name2str(ub) << "=" << ub << std::endl;
	car_kin_log_ << name2str(lb) << "=" << lb << std::endl;
	car_kin_log_ << name2str(s_lb) << "=" << s_lb << std::endl;
	car_kin_log_ << name2str(s_ub) << "=" << s_ub << std::endl;
}

void controller::CarKinMPC::Init(MPCConfig mpc_config, CarParm param)
{
	wheel_base_ = param.wheel_base;
	max_delta_f_ = param.max_delta_f;
	max_delta_f_rate_ = param.max_delta_f_rate;
	max_v_ = param.max_v;
	min_v_ = param.min_v;
	max_acc_ = param.max_acc;
	max_decel_ = param.max_decel;
	Init(mpc_config);
}

std::string controller::CarKinMPC::ControlerInfo()
{
	return controller_info_.str();
}

void controller::CarKinMPC::CurrentState(Eigen::MatrixXd state)
{
	state_ = state.block(0,0,3,1);
	matrix_u_ = state.block(3, 0, 2, 1);
}

Eigen::MatrixXd controller::CarKinMPC::ComputeControlCommand(Eigen::MatrixXd ref)
{
	assert(ref.rows() == 3*mpc_config_.np);
	int nu = 2, ns = 3, ny = 3;
	CalStateFunc();
	Matrix a2, b2(ns + nu,nu), c2;
	a2 = Matrix::Zero(ns + nu, ns + nu);
	a2.block(0, 0, ns, ns) = matrix_a_;
	a2.block(0, ns, ns, nu) = matrix_b_;
	a2.block(ns, ns, nu, nu) = Matrix::Identity(nu, nu);
	b2.block(0, 0, ns, nu) = matrix_b_;
	b2.block(ns, 0, nu, nu) = Matrix::Identity(nu, nu);
	c2 = Matrix::Zero(ny, ns + nu);
	c2.block(0, 0, ny, ns) = matrix_c_;

	mpc_config_.x0 = Matrix(ns + nu, 1);
	mpc_config_.x0.block(0, 0, ns, 1) = state_;
	mpc_config_.x0.block(ns, 0, nu, 1) = matrix_u_;

	mpc_config_.u0=matrix_del_u_;
	
	ControlConf conf;
	conf.matrix_a = a2;
	conf.matrix_b = b2;
	conf.matrix_c = c2;
	conf.ref = ref;
	conf.q = mpc_config_.q;
	conf.r = mpc_config_.r;
	conf.x0 = mpc_config_.x0;
	conf.u0 = mpc_config_.u0;
	mpcslover_.LoadControlConf(conf);

	Matrix result = mpcslover_.GetControlCommand();

	matrix_del_u_ = result.block(0, 0, nu, 1);
	if (matrix_del_u_.size()<=0)
	{
		matrix_del_u_ = Matrix::Zero(nu, 1);
	}
	matrix_u_ += matrix_del_u_;	

	car_kin_log_ << name2str(a2) << "=" << a2 << std::endl;
	car_kin_log_ << name2str(b2) << "=" << b2 << std::endl;
	car_kin_log_ << name2str(c2) << "=" << c2 << std::endl;
	car_kin_log_ << name2str(mpc_config_.x0) << "=" << mpc_config_.x0 << std::endl;
	car_kin_log_ << name2str(mpc_config_.u0) << "=" << mpc_config_.u0 << std::endl;
	car_kin_log_ << "delta_U" << "=" << matrix_del_u_ << std::endl;
	car_kin_log_ << name2str(matrix_u_) << "=" << matrix_u_ << std::endl;
	return matrix_u_;
}

void controller::CarKinMPC::CalStateFunc()
{
	double v = matrix_u_(0), delta_f = matrix_u_(1);
	double theta = state_(2);
	matrix_a_ = Matrix(3, 3);
	matrix_b_ = Matrix(3, 2);
	matrix_c_ = Matrix::Identity(3, 3);
	
	matrix_a_ << 0, 0, -v * sin(theta),
		0, 0, v*cos(theta),
		0, 0, 0;
	matrix_b_ << cos(theta), 0,
		sin(theta), 0,
		tan(delta_f) / wheel_base_, (v*(tan(delta_f)*tan(delta_f) + 1)) / wheel_base_;

	car_kin_log_ << "====State Funciton=======" << std::endl;
	car_kin_log_ << name2str(matrix_a_) << "=" << matrix_a_ << std::endl;
	car_kin_log_ << name2str(matrix_b_) << "=" << matrix_b_ << std::endl;
	car_kin_log_ << name2str(matrix_c_) << "=" << matrix_c_ << std::endl;
}
