#ifndef DOMAIN_MISSION__HXX
#define DOMAIN_MISSION__HXX

#include <memory>
#include <ktp_data_msgs/msg/mission.hpp>

namespace ktp
{
    namespace domain
    {
        class Mission final
        {
        private:
            u_int16_t mission_status_code_;
            ktp_data_msgs::msg::Mission mission_;
        public:
            explicit Mission();
            virtual ~Mission();
            u_int16_t  get__mission_status_code();
            void set__mission_status_code(u_int16_t mission_status_code);
            ktp_data_msgs::msg::Mission get__mission();
            void set__mission(ktp_data_msgs::msg::Mission mission);
        public:
            using SharedPtr = std::shared_ptr<Mission>;
            using UniquePtr = std::unique_ptr<Mission>;
        };
    }
}

#endif // DOMAIN_MISSION__HXX