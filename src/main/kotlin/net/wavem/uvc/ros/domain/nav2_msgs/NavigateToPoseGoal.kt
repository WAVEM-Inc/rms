package net.wavem.uvc.ros.domain.nav2_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.geometry_msgs.PoseStamped

class NavigateToPoseGoal() : Message {
    var pose : PoseStamped = PoseStamped()
    var behavior_tree : String = ""
}