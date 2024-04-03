//
// Created by reidlo on 24. 3. 7.
//

#ifndef OBSTACLE_DETECT_MANAGER_HXX
#define OBSTACLE_DETECT_MANAGER_HXX

#include <rclcpp/rclcpp.hpp>

namespace ktp
{
    namespace data
    {
        class ObstacleDetectManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit ObstacleDetectManager(rclcpp::Node::SharedPtr node);
            virtual ~ObstacleDetectManager();
        public:
            using SharedPtr = std::shared_ptr<ObstacleDetectManager>;
        };
    }
}

#endif // OBSTACLE_DETECT_MANAGER_HXX
