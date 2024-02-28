#ifndef ASSIGNMENT_EXCEPTION__HXX
#define ASSIGNMENT_EXCEPTION__HXX

#include <iostream>
#include <exception>

#define CSTR(str) ((str).c_str())

namespace ktp
{
    namespace exceptions
    {
        class DataOmissionException : public std::exception
        {
        private:
            std::string message;

        public:
            explicit DataOmissionException(const std::string &msg);
            virtual ~DataOmissionException();

            const char *what() const noexcept override;
        };
    }
}

#endif // ASSIGNMENT_EXCEPTION__HXX