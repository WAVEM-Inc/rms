package net.wavem.uvc.ros.slam.application.math

import net.wavem.uvc.ros.slam.domain.GTSPoint
import kotlin.math.acos
import kotlin.math.sqrt

object GTSMath {
    fun calculateRotation(gtsPoint : GTSPoint) : Double {
        val x1 : Double = gtsPoint.slamPoint1.x
        val y1 : Double = gtsPoint.slamPoint1.y
        val x2 : Double = gtsPoint.slamPoint2.x
        val y2 : Double = gtsPoint.slamPoint2.y
        val x3 : Double = gtsPoint.ktmPoint1.x
        val y3 : Double = gtsPoint.ktmPoint1.y
        val x4 : Double = gtsPoint.ktmPoint2.x
        val y4 : Double = gtsPoint.ktmPoint2.y

        val vABx : Double = x2 - x1
        val vABy : Double = y2 - y1
        val vCDx : Double = x4 - x3
        val vCDy : Double = y4 - y3

        val dotProduct : Double = vABx * vCDx + vABy * vCDy

        val magnitudeAB : Double = sqrt(vABx * vABx + vABy * vABy)
        val magnitudeCD : Double = sqrt(vCDx * vCDx + vCDy * vCDy)

        return acos(dotProduct / (magnitudeAB * magnitudeCD))
    }
}
