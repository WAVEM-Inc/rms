package net.wavem.uvc.ros.nav_msgs.srv

import net.wavem.uvc.ros.nav_msgs.msg.map.OccupancyGrid

data class GetMap(
    val map: OccupancyGrid
) {
}