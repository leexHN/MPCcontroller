#pragma once
#include <string>
#include "text_logger.h"
#include <Eigen/Dense>

struct ControlConf
{
	Eigen::MatrixXd matrix_a, matrix_b, matrix_c, ref, q, r, x0, u0;
	double rho=0; //松弛因子权重
};

class MPCSlover
{
public:
	MPCSlover(text_logger* logger);

	MPCSlover() = default;

	virtual ~MPCSlover()=default;

	void Init(size_t np,size_t nc, double ts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s_lb, Eigen::MatrixXd s_ub);

	void Init(size_t np, size_t nc, double ts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s_lb, Eigen::MatrixXd s_ub,
		Eigen::MatrixXd y_lb, Eigen::MatrixXd y_ub, double slack_para);

	bool LoadControlConf(ControlConf config);

	Eigen::MatrixXd GetControlCommand();

	void operator() (text_logger* logger);

protected:
	
	text_logger *p_text_logger_;
	
	size_t np_, nc_, ns_, nu_, ny_;

	Eigen::MatrixXd matrix_ad_, matrix_bd_, matrix_cd_;

	Eigen::MatrixXd matrix_the_, matrix_psi_, matrix_z_;

	Eigen::MatrixXd matrix_q_, matrix_r_;

	Eigen::MatrixXd matrix_state_;

	Eigen::MatrixXd matrix_lb_, matrix_ub_;

	Eigen::MatrixXd matrix_s_lb_, matrix_s_ub_;

	Eigen::MatrixXd matrix_ref_;

	Eigen::MatrixXd matrix_h_, matrix_f_;

	Eigen::MatrixXd matrix_m1_, matrix_m2_;

	Eigen::MatrixXd matrix_d1_, matrix_d2_;

	Eigen::MatrixXd matrix_y_lb_, matrix_y_ub_;

	Eigen::MatrixXd result_;

	double slack_ub_,slack_rho_;

	void InitControlBoundaryConditions();

	Eigen::MatrixXd NoSlackQpSlover(Eigen::MatrixXd mu);

	Eigen::MatrixXd SlackQpSlover(Eigen::MatrixXd mu ,Eigen::MatrixXd x0);

	double ts_;
};
