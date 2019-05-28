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
	slover.Init(np, nc, lb, lb, lb, lb);
}