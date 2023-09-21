package net.wavem.uvc.ros.sensor_msgs.msg.gps

data class NavSatStatus(
    val STATUS_NO_FIX : Byte = -1,
    val STATUS_FIX : Byte = 0,
    val STATUS_SBAS_FIX : Byte = 1,
    val STATUS_GBAS_FIX : Byte = 2,
    val status : Byte,
    val SERVICE_GPS : Short = 1,
    val SERVICE_GLONASS : Short = 2,
    val SERVICE_COMPASS : Short = 4,
    val SERVICE_GALILEO : Short = 8,
    val service : Short
) {
}