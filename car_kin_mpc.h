#pragma once
#include "mpc_controller.h"
#include <sstream>
#include "text_logger.h"
#include <string>
#include "mpc_solver.h"

#define DEG2RAD(x) double(x)/180.0*3.1415926

namespace controller {
	struct CarParm
	{
		double wheel_base;

		double max_delta_f, max_delta_f_rate;

		double max_v, min_v, max_acc, max_decel;
	};

	class CarKinMPC : public MPCController
	{
	public:
		CarKinMPC();
		~CarKinMPC() = default;

		void Init(MPCConfig mpc_config) override;

		void Init(MPCConfig mpc_config, CarParm parm);

		std::string ControlerInfo() override;

		void CurrentState(Eigen::MatrixXd state) override;

		Eigen::MatrixXd ComputeControlCommand(Eigen::MatrixXd ref) override;
		
	private:
		void CalStateFunc();

		text_logger solver_log_;

		text_logger car_kin_log_;

		MPCSlover mpcslover_;

		std::stringstream controller_info_;

		MPCConfig mpc_config_;

		Eigen::MatrixXd state_;

		Eigen::MatrixXd matrix_u_; // 实际车辆的control value

		Eigen::MatrixXd matrix_a_, matrix_b_, matrix_c_; //实际车辆的状态方程

		Eigen::MatrixXd matrix_del_u_; // mapc solver的控制量

		double wheel_base_=1.0;

		double max_delta_f_, max_delta_f_rate_;

		double max_v_, min_v_ ,max_acc_, max_decel_;

		double ts_;
	};

}