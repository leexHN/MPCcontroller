#include <iostream>
#include "text_logger.h"
#include "mpc_controller.h"
#include <stdlib.h>


int main()
{
	text_logger logger;

	std::cout << "Hello World!\n";
	
	MPCSlover slover(&logger);
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
	slover.LoadControlConf(conf);
	system("pause");
}