package net.wavem.uvc.ros.geometry_msgs.msg

data class PoseWithCovariance(
    val pose: Pose,
    val covariance: DoubleArray = DoubleArray(36)
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as PoseWithCovariance

        return covariance.contentEquals(other.covariance)
    }

    override fun hashCode(): Int {
        return covariance.contentHashCode()
    }

}