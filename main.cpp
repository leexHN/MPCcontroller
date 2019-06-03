#include <iostream>
#include "text_logger.h"
#include "mpc_solver.h"
#include <stdlib.h>
#include "car_kin_mpc.h"

int main()
{
	text_logger logger;

	std::cout << "Hello World!\n";
	
	MPCSlover slover;
	slover(&logger);
	
	size_t np = 10, nc = 2;
	Eigen::MatrixXd lb(1, 1),ub(1, 1), s_lb(1,1),s_ub(1,1);
	lb << -1;
	ub << 2;
	s_lb << -5;
	s_ub << 10;
	slover.Init(np, nc, 0.1, lb, ub, s_lb, s_ub);
	Eigen::MatrixXd a(2, 2), b(2, 1), c(1, 2), x0(2, 1), u0(1,1);
	a << 0, 1,
		-1, -2;
	b << 0,
		1;
	c << 1, 0;
	x0 << 1,
		2;
	u0 << 1;
	ControlConf conf;
	conf.matrix_a = a;
	conf.matrix_b = b;
	conf.matrix_c = c;
	conf.ref = 4*Eigen::MatrixXd::Ones(np, 1);
	conf.q = 2*Eigen::MatrixXd::Identity(1, 1); // 一个输出
	conf.r = 3*Eigen::MatrixXd::Identity(1, 1); //1个输入
	conf.x0 = x0;
	conf.u0 = u0;
	slover.LoadControlConf(conf);
	// slack test
	Eigen::MatrixXd y_lb(1,1), y_ub(1,1);
	y_lb << -1;
	y_ub << 5;
	slover.Init(np, nc, 0.1, lb, ub, s_lb, s_ub, y_lb, y_ub,20.0);
	conf.rho = 1000;
	if (slover.LoadControlConf(conf))
	{
		std::cout << "succeed\n"<<slover.GetControlCommand()<<std::endl;
	}
	else
	{
		std::cout << "\nfailed\n";
	}

	// car_kin_test
	controller::CarKinMPC car_kin_mpc;

	controller::CarParm car_parm;
	car_parm.wheel_base = 2.6;
	car_parm.max_delta_f = DEG2RAD(30);
	car_parm.max_delta_f_rate = DEG2RAD(15);
	car_parm.max_v = 5;
	car_parm.min_v = -5;
	car_parm.max_acc = 1;
	car_parm.max_decel = -1;

	controller::MPCConfig mpc_config;
	mpc_config.np = 10;
	mpc_config.nc = 2;
	mpc_config.ts = 0.1;
	mpc_config.q =10* Eigen::MatrixXd::Identity(3,3);
	mpc_config.r = Eigen::MatrixXd::Identity(2, 2);;

	Eigen::MatrixXd ini_state(5, 1);
	ini_state << 0, 0, 0,0.5,0;

	car_kin_mpc.Init(mpc_config, car_parm);
	car_kin_mpc.CurrentState(ini_state);

	Eigen::MatrixXd ref(3 * mpc_config.np, 1);
	for (size_t i = 0; i < mpc_config.np; i++)
	{
		Eigen::MatrixXd temp(3, 1);
		temp << 5, 1, 0;
		ref.block(i * 3, 0, 3, 1) = temp;
	}
	car_kin_mpc.ComputeControlCommand(ref);

	
	
	
	system("pause");
}