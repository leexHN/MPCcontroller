#pragma once
#include <string>
#include "text_logger.h"
#include <Eigen/Dense>

struct ControlConf
{
	Eigen::MatrixXd matrix_a, matrix_b, matrix_c, ref, q, r, x0, u0;
	double rho=0; //ËÉ³ÚÒò×Ó
};

class MPCSlover
{
public:
	MPCSlover(text_logger* logger);

	virtual ~MPCSlover()=default;

	void Init(size_t np,size_t nc, double ts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s_lb, Eigen::MatrixXd s_ub);

	bool LoadControlConf(ControlConf config);

	Eigen::MatrixXd ComputeControlCommand();

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

	void GetBoundaryConditions();

	void InitControlBoundaryConditions();

	void NoSlackQpSlover(Eigen::MatrixXd mu);

	double ts_;
};
