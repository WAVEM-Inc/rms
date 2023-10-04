package net.wavem.uvc.ros.slam.application.math

import net.wavem.uvc.ros.slam.domain.SLAMPoint
import kotlin.math.cos
import kotlin.math.sin

object SLAMMath {
    fun distanceFormula(startPoint : SLAMPoint, endPoint : SLAMPoint) : Double {
        return CommonMath.distanceFormula(startPoint.x, startPoint.y, endPoint.x, endPoint.y)
    }

    fun angleBetweenTwoPoints(startPoint : SLAMPoint, endPoint : SLAMPoint) : Double {
        return CommonMath.angleBetweenTwoPoints(startPoint.x, startPoint.y, endPoint.x, endPoint.y)
    }

    fun rotatePoint(x : Double, y : Double, angle : Double) : SLAMPoint {
        val xNew : Double = x * cos(angle) - y * sin(angle)
        val yNew : Double = x * sin(angle) + y * cos(angle)

        return SLAMPoint(xNew, yNew)
    }
}
