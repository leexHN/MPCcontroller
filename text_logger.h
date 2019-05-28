#pragma once

#include <string>
#include <fstream>
#include <iostream>


using namespace std;

class text_logger
{
public:
	text_logger();
	text_logger(std::string file_name);
	~text_logger();
	template <typename T>
	ofstream& operator<<(const T data);
	void close();
private:
	std::string file_name_;
	std::ofstream out_;
	void init();

};

template<typename T>
inline ofstream & text_logger::operator<<(const T data)
{
	out_ << data;
	return out_;
}
