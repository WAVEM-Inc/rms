#ifndef DOMAIN_MISSION__HXX
#define DOMAIN_MISSION__HXX

#include <memory>

#include <ktp_data_msgs/msg/mission.hpp>
#include <route_msgs/msg/node.hpp>

#define NAVIGATION_STATUS_WAITING_CODE 0

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

        class NavigationStatus final
        {
        private:
            int drive_status_;
            route_msgs::msg::Node start_node_;
            route_msgs::msg::Node end_node_;
            ktp_data_msgs::msg::MissionTask mission_task_;
        public:
            explicit NavigationStatus();
            virtual ~NavigationStatus();
            int get__drive_status();
            void set__drive_status(int drive_status);
            route_msgs::msg::Node get__start_node();
            void set__start_node(route_msgs::msg::Node start_node);
            route_msgs::msg::Node get__end_node();
            void set__end_node(route_msgs::msg::Node end_node);
            ktp_data_msgs::msg::MissionTask get__mission_task();
            void set__mission_task(ktp_data_msgs::msg::MissionTask mission_task);
        public:
            using SharedPtr = std::shared_ptr<NavigationStatus>;
        };
    }
}

#endif // DOMAIN_MISSION__HXX