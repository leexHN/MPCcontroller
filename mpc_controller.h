#pragma once
#include <string>
#include "text_logger.h"
#include <Eigen/Core>

class MPCSlover
{
public:
	MPCSlover(text_logger* logger);

	virtual ~MPCSlover()=default;

	void Init(size_t np,size_t nc, double ts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s_lb, Eigen::MatrixXd s_ub);

	bool LoadControlConf(Eigen::MatrixXd matrix_a, Eigen::MatrixXd matrix_b, Eigen::MatrixXd matrix_c, Eigen::MatrixXd ref);

protected:
	
	text_logger *p_text_logger_;
	
	size_t np_, nc_, n_state_,nu_;

	Eigen::MatrixXd matrix_ad_, matrix_bd_, matrix_cd_;

	Eigen::MatrixXd matix_s_, matrix_t_;

	Eigen::MatrixXd matrix_state_;

	Eigen::MatrixXd matrix_lb_, matrix_ub_;

	Eigen::MatrixXd matrix_s_lb_, matrix_s_ub_;

	Eigen::MatrixXd matrix_ref_;

	double ts_;
};
