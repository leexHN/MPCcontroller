#include "mpc_controller.h"
#include "active_set_qp_solver.h"
#include <vector>
#include <memory>

#define name2str(name) (#name)

using Matrix = Eigen::MatrixXd;

Matrix kroneckerProduct(const Matrix m1, const Matrix m2);

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
	InitControlBoundaryConditions();
}

bool MPCSlover::LoadControlConf(ControlConf config)
{
	if (config.matrix_a.rows() != config.matrix_a.cols() ||
		config.matrix_b.rows() != config.matrix_a.rows()||
		config.matrix_c.cols() != config.matrix_a.rows() ||
		config.ref.rows() != np_* config.matrix_c.rows()) {
		std::cout << "One or more matrices have incompatible dimensions. Aborting.";
		return false;
	}
	matrix_ref_ = static_cast<Matrix>(config.ref);

	ns_ = config.matrix_a.rows();
	ny_ = config.matrix_c.rows();
	
	// 离散化状态方程
	Matrix matrix_i = Matrix::Identity(config.matrix_a.cols(), config.matrix_a.cols());
	matrix_ad_ = (matrix_i - ts_ * 0.5 * config.matrix_a).inverse() *
		(matrix_i + ts_ * 0.5 * config.matrix_a);
	matrix_bd_ = ts_ * config.matrix_b;
	matrix_cd_ = static_cast<Matrix>(config.matrix_c);

	// 建立相应的矩阵
	std::vector<Matrix> matrix_a_power(np_+1);
	matrix_a_power[0] = Matrix::Identity(ns_,ns_);
	for (size_t i = 1; i < matrix_a_power.size(); ++i) {
		matrix_a_power[i] = matrix_ad_ * matrix_a_power[i - 1];
	} // temp  matrix_t
	
	matrix_psi_ = Matrix::Zero(ny_ * np_, ns_);
	for (size_t row = 0; row < np_; row++)
	{
		matrix_psi_.block(row*ny_, 0, ny_, ns_) = matrix_cd_ * matrix_a_power.at(row+1);
	}

	matrix_the_ = Matrix::Zero(np_*ny_, nu_*nc_);
	for (size_t row = 0; row < np_; row++)
	{
		for (size_t col = 0; col < nc_; col++)
		{
			if (row >= col)
			{
				matrix_the_.block(row*ny_, nu_*col, ny_, nu_) = matrix_cd_ * matrix_a_power.at(row - col)*matrix_bd_;
			}
		}
	}

	matrix_q_ = Matrix::Zero(ny_*np_, ny_*np_);
	matrix_r_ = Matrix::Zero(nc_*nu_, nc_*nu_);
	for (size_t row = 0; row < np_; row++)
	{
		matrix_q_.block(row*ny_, row*ny_, ny_, ny_) = config.q;
	}
	for (size_t row = 0; row < nc_; row++)
	{
		matrix_r_.block(row*nu_, row*nu_, nu_, nu_) = config.r;
	}

	*(p_text_logger_) << name2str(ns_) << "=" << ns_ << endl;
	*(p_text_logger_) << name2str(nu_) << "=" << nu_ << endl;
	*(p_text_logger_) << name2str(ny_) << "=" << ny_ << endl;
	*(p_text_logger_) << name2str(matrix_lb_) << "=" << matrix_lb_ << endl;
	*(p_text_logger_) << name2str(matrix_ub_) << "=" << matrix_ub_ << endl;
	*(p_text_logger_) << name2str(matrix_s_lb_) << "=" << matrix_s_lb_ << endl;
	*(p_text_logger_) << name2str(matrix_s_ub_) << "=" << matrix_s_ub_ << endl;
	*(p_text_logger_) << "======load control config=======" << endl;
	*(p_text_logger_) << name2str(config.matrix_a) << "=" << config.matrix_a << endl;
	*(p_text_logger_) << name2str(config.matrix_b) << "=" << config.matrix_b << endl;
	*(p_text_logger_) << name2str(config.matrix_c) << "=" << config.matrix_c << endl;
	*(p_text_logger_) << name2str(config.x0) << "=" << config.x0 << endl;
	*(p_text_logger_) << name2str(matrix_ref_) << "=" << matrix_ref_ << endl;
	*(p_text_logger_) << name2str(matrix_ad_) << "=" << matrix_ad_ << endl;
	*(p_text_logger_) << name2str(matrix_bd_) << "=" << matrix_bd_ << endl;
	*(p_text_logger_) << name2str(matrix_cd_) << "=" << matrix_cd_ << endl;

	*(p_text_logger_) << name2str(matrix_psi_) << "=" << matrix_psi_ << endl;
	*(p_text_logger_) << name2str(matrix_the_) << "=" << matrix_the_ << endl;
	*(p_text_logger_) << name2str(matrix_q_) << "=" << matrix_q_ << endl;
	*(p_text_logger_) << name2str(matrix_r_) << "=" << matrix_r_ << endl;

	matrix_h_ = matrix_the_.transpose()*matrix_q_*matrix_the_ + matrix_r_;
	matrix_f_ = 2 * (matrix_psi_*config.x0 - matrix_ref_).transpose()*matrix_q_*matrix_the_;

	*(p_text_logger_) << "======create QP parameter=======" << endl;
	*(p_text_logger_) << name2str(matrix_h_) << "=" << matrix_h_ << endl;
	*(p_text_logger_) << name2str(matrix_f_) << "=" << matrix_f_ << endl;

	Matrix mu = Matrix::Zero(2 * nu_*nc_, 1);
	for (size_t i = 0; i < 2 * nc_; i++)
	{
		i< nc_ ? mu.block(i, 0, nu_, 1) = config.u0 : mu.block(i, 0, nu_, 1) = -config.u0;
	}

	if (config.rho <= 0.0)
	{
		NoSlackQpSlover(mu);
	}

	return true;
}

Eigen::MatrixXd MPCSlover::ComputeControlCommand()
{
	//std::unique_ptr<math::QpSolver>qp_solver
	return Eigen::MatrixXd();
}

void MPCSlover::InitControlBoundaryConditions()
{
	// control_value
	matrix_m1_ =  Matrix::Zero(2 * nc_*nu_, nu_*nc_); 
	matrix_m1_.block(0, 0, nc_*nu_, nc_*nu_) = Matrix::Identity(nu_*nc_, nu_*nc_);
	matrix_m1_.block(nc_*nu_, 0, nc_*nu_, nc_*nu_) = -1 * Matrix::Identity(nc_*nu_, nc_*nu_);
	
	matrix_d1_ = Matrix::Zero(2 * nc_*nu_, 1);
	matrix_d1_.block(0, 0, nc_*nu_, 1) = matrix_ub_;
	matrix_d1_.block(nc_*nu_, 0, nc_*nu_, 1) = -matrix_lb_;

	// sum of control_value
	Matrix temp_m1 = Matrix::Identity(nc_, nc_);
	for (size_t row = 0; row < nc_; row++)
	{
		for (size_t col = 0; col < nc_; col++)
		{
			if (col < row)
				temp_m1(row, col) = 1;
		}
	}
	Matrix temp_m2 = kroneckerProduct(temp_m1, Matrix::Identity(nu_, nu_));
	Matrix m2(2 * nc_*nu_, nc_*nu_);
	m2.block(0, 0, nc_*nu_, nc_*nu_) = temp_m2;
	m2.block(nc_*nu_, 0, nc_*nu_, nc_*nu_) = -temp_m2;
	matrix_m2_ = m2;

	matrix_d2_ = Matrix::Zero(2 * nc_*nu_, 1);
	matrix_d2_.block(0, 0, nc_*nu_, 1) = matrix_s_ub_;
	matrix_d2_.block(nc_*nu_, 0, nc_*nu_, 1) = -matrix_s_lb_;
}


void MPCSlover::NoSlackQpSlover( Eigen::MatrixXd mu)
{
	// add inequality_constrain
	Matrix qp_c((4 * nc_ * nu_), (nc_ * nu_));
	qp_c.block(0, 0, (2 * nc_*nu_), (nc_*nu_)) = matrix_m1_;
	qp_c.block((2 * nc_*nu_), 0, (2 * nc_*nu_), (nc_*nu_)) = matrix_m2_;
	
	Matrix qp_d((4 * nc_*nu_), 1);
	qp_d.block(0, 0, (2 * nc_*nu_), 1) = matrix_d1_;
	qp_d.block((2 * nc_*nu_), 0, (2 * nc_*nu_), 1) = matrix_d2_ - mu;

	// add equality_constrain
	Matrix matrix_equality_constrain = Matrix::Zero(nu_*nc_, nu_*nc_);
	Matrix matrix_equality_boundary = Matrix::Zero(nu_*nc_, 1);

	std::unique_ptr<math::QpSolver> qp_solver(new math::ActiveSetQpSolver(2 * matrix_h_, matrix_f_.transpose(),
		-qp_c, -qp_d,
		matrix_equality_constrain, matrix_equality_boundary));
	auto result = qp_solver->Solve();
	if (!result) {
		std::cout << "Linear MPC solver failed";
		return;
	}
	Matrix matrix_v = Matrix::Zero(nu_ * nc_, 1);
	matrix_v = qp_solver->params();
	std::cout << "result is:" << std::endl << matrix_v << std::endl;

	*(p_text_logger_) << name2str(mu) << "=" << mu << endl;
	*(p_text_logger_) << name2str(qp_c) << "=" << qp_c << endl;
	*(p_text_logger_) << name2str(qp_d) << "=" << qp_d << endl;
	*(p_text_logger_) << name2str(matrix_equality_constrain) << "=" << matrix_equality_constrain << endl;
	*(p_text_logger_) << name2str(matrix_equality_boundary) << "=" << matrix_equality_boundary << endl;
	*(p_text_logger_) << name2str(matrix_v) << "=" << matrix_v << endl;
}


Matrix kroneckerProduct(const Matrix m1, const Matrix m2) {
	Matrix m3(m1.rows()*m2.rows(), m1.cols()*m2.cols());
	for (int i = 0; i < m1.rows(); i++) {
		for (int j = 0; j < m1.cols(); j++) {
			m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) = m1(i, j)*m2;
		}
	}
	return m3;
}


