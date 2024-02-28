#include "exception/assignment/assignment_exceptions.hxx"

ktp::exceptions::DataOmissionException::DataOmissionException(const std::string &msg)
    : message(msg)
{
}

ktp::exceptions::DataOmissionException::~DataOmissionException()
{
}

const char *ktp::exceptions::DataOmissionException::what() const noexcept 
{
    return CSTR(this->message);
}