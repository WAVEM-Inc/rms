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
            uint8_t response_code_;
            uint8_t status_code_;
            ktp_data_msgs::msg::Mission mission_;

        public:
            explicit Mission();
            virtual ~Mission();
            uint8_t get__response_code();
            void set__response_code(uint8_t response_code);
            uint8_t get__status_code();
            void set__status_code(uint8_t mission_status_code);
            ktp_data_msgs::msg::Mission get__mission();
            void set__mission(ktp_data_msgs::msg::Mission mission);

        public:
            using SharedPtr = std::shared_ptr<Mission>;
            using UniquePtr = std::unique_ptr<Mission>;
        };
    }
}

#endif // DOMAIN_MISSION__HXX