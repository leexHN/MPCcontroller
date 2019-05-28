
#include "text_logger.h"
#include <ctime>




text_logger::text_logger()
{
	file_name_ = "logger.log";
	init();
}

text_logger::text_logger(std::string file_name) :file_name_(file_name) {
	init();
}

text_logger::~text_logger()
{
	out_.close();
}

void text_logger::close()
{
	out_.close();
}

void text_logger::init()
{
	bool is_exist = true;
	out_.open(file_name_, ios::in);
	if (!out_)is_exist = false;
	out_.close();
	out_.open(file_name_.c_str(), ios::out | ios::app);
	if (is_exist) out_ << endl;
	time_t now = time(0);
	//tm *ltm = localtime(&now);
	//out_ << "=="<<ltm->tm_year<<"/"<<ltm->tm_mon<<"/"<<ltm->tm_mday<<"/"<<ltm->tm_hour<<"/"<<ltm->tm_min<<"/"<<ltm->tm_sec<<"==";
	out_ << "======start:" << now << "========" << endl;
};



