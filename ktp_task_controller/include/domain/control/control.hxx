#ifndef DOMAIN_CONTROL__HXX
#define DOMAIN_CONTROL__HXX

#include <memory>
#include <ktp_data_msgs/msg/control.hpp>

namespace ktp
{
    namespace domain
    {
        class Control final
        {
        private:
            ktp_data_msgs::msg::Control control_;
        public:
            explicit Control();
            virtual ~Control();
            ktp_data_msgs::msg::Control get__control();
            void set__control(ktp_data_msgs::msg::Control control);
        public:
            using SharedPtr = std::shared_ptr<Control>;
            using UniquePtr = std::unique_ptr<Control>;
        };
    }
}

#endif // DOMAIN_CONTROL__HXX