#include "mpc_controller.h"


#define name2str(name) (#name)

using Matrix = Eigen::MatrixXd;


MPCSlover::MPCSlover(text_logger * logger)
{
	p_text_logger_ = logger;
}

void MPCSlover::Init(size_t np, size_t nc,double ts, Eigen::MatrixXd lb, Eigen::MatrixXd ub, Eigen::MatrixXd s_lb, Eigen::MatrixXd s_ub)
{
	np_ = np;
	nc_ = nc;
	nu_ = lb.rows();
	ts_ = ts;

	*(p_text_logger_) << name2str(np_) << "=" << np_ << endl;
	*(p_text_logger_) << name2str(nc_) << "=" << nc_ << endl;
	*(p_text_logger_) << name2str(nu_) << "=" << nu_ << endl;
	*(p_text_logger_) << name2str(ts_) << "=" << ts_ << endl;

	matrix_lb_ = Matrix::Zero(nc_ * nu_, 1);
	matrix_ub_ = Matrix::Zero(nc_ * nu_, 1);
	matrix_s_lb_ = Matrix::Zero(nc_ * nu_, 1);
	matrix_s_ub_ = Matrix::Zero(nc_ * nu_, 1);

	for (size_t i = 0; i < nc_; i++)
	{
		matrix_lb_.block(i*nu_, 0, nu_, 1) = lb;
		matrix_ub_.block(i*nu_, 0, nu_, 1) = ub;
		matrix_s_lb_.block(i*nu_, 0, nu_, 1) = s_lb;
		matrix_s_ub_.block(i*nu_, 0, nu_, 1) = s_ub;
	}

	*(p_text_logger_) << name2str(matrix_lb_) << "=" << matrix_lb_ << endl;
	*(p_text_logger_) << name2str(matrix_ub_) << "=" << matrix_ub_ << endl;
	*(p_text_logger_) << name2str(matrix_s_lb_) << "=" << matrix_s_lb_ << endl;
	*(p_text_logger_) << name2str(matrix_s_ub_) << "=" << matrix_s_ub_ << endl;
}

bool MPCSlover::LoadControlConf(Eigen::MatrixXd matrix_a, Eigen::MatrixXd matrix_b, Eigen::MatrixXd matrix_c, Eigen::MatrixXd ref)
{
	if (matrix_a.rows() != matrix_a.cols() ||
		matrix_b.rows() != matrix_a.rows()||
		ref.cols() != np_*matrix_c.rows()) {
		std::cout << "One or more matrices have incompatible dimensions. Aborting.";
		return false;
	}
	matrix_ref_ = static_cast<Matrix>(ref);
	
	Matrix matrix_i = Matrix::Identity(matrix_a.cols(), matrix_a.cols());
	matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a).inverse() *
		(matrix_i + ts_ * 0.5 * matrix_a);
	matrix_bd_ = Matrix::Zero(n_state_, nu_);
	matrix_cd_ = static_cast<Matrix>(matrix_c);
}


