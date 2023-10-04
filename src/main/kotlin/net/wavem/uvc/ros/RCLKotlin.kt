package net.wavem.uvc.ros

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.boot.CommandLineRunner
import org.springframework.stereotype.Component
import us.ihmc.pubsub.DomainFactory
import us.ihmc.ros2.ROS2Node

@Component
class RCLKotlin : CommandLineRunner {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)
    private var node : ROS2Node? = null

    fun rclKotlinInit() {
        this.node = ROS2Node(DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS), "uvc_manufacture_server")

        if (node != null) {
            logger.info("RCLKotlin initialized")
        } else {
            logger.error("Failed to initialize RCLKotlin")
        }
    }

    fun getRCLNode() : ROS2Node {
        return this.node!!
    }

    fun isRCLKotlinInitialized() : Boolean {
        return (node != null)
    }

    override fun run(vararg args : String?) {
        this.rclKotlinInit()
    }
}