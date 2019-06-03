#pragma once

#include <Eigen/Dense>
#include <string>

namespace controller
{

	struct MPCConfig
	{
		size_t np, nc;
		double ts;
		double rho = 0.0;
		Eigen::MatrixXd q, r; // È¨ÖØ
		Eigen::MatrixXd x0, u0; //³õÊ¼×´Ì¬
	};

	class MPCController
	{
	public:

		MPCController() = default;

		virtual ~MPCController() = default;

		virtual void Init(MPCConfig mpc_config)=0;

		virtual std::string ControlerInfo() = 0;

		virtual void CurrentState(Eigen::MatrixXd state) = 0;

		virtual Eigen::MatrixXd ComputeControlCommand(Eigen::MatrixXd ref) = 0;
	};

}