package net.wavem.uvc.ros2.nav_msgs.api.map.response

import net.wavem.uvc.ros2.std_msgs.msg.String
import org.springframework.stereotype.Component

@Component
class MapResponseHandler {

    fun handle(message: String) {
        println("message arrived : $message")
    }
}