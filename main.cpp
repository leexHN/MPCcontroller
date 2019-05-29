#include <iostream>
#include "text_logger.h"
#include "mpc_controller.h"

int main()
{
	text_logger logger;

	std::cout << "Hello World!\n";
	
	MPCSlover slover(&logger);
	size_t np = 10, nc = 2;
	Eigen::MatrixXd lb(3, 1);
	lb << 1, 2,3;
	slover.Init(np, nc, 0.1, lb, lb, lb, lb);
	Eigen::MatrixXd a(2, 2),b(2,1),c(1,2),x0(2,1); 
	a << 0, 1,
		-1, -2;
	b << 0,
		1;
	c << 1, 0;
	x0 << 1,
		2;
	ControlConf conf;
	conf.matrix_a = a;
	conf.matrix_b = b;
	conf.matrix_c = c;
	conf.ref = 4*Eigen::MatrixXd::Ones(np, 1);
	conf.q = 2*Eigen::MatrixXd::Identity(1, 1); // 一个输出
	conf.r = 3*Eigen::MatrixXd::Identity(1, 1); //1个输入
	conf.x0 = x0;
	slover.LoadControlConf(conf);
}