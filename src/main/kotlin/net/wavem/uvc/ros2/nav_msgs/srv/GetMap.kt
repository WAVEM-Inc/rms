package net.wavem.uvc.ros2.nav_msgs.srv

import net.wavem.uvc.ros2.nav_msgs.msg.OccupancyGrid

data class GetMap(
    val map: OccupancyGrid
) {
}