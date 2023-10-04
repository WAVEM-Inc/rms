package net.wavem.uvc.ros.slam.application.math

import kotlin.math.atan2
import kotlin.math.sqrt

object CommonMath {
    fun distanceFormula(x1 : Double, y1 : Double, x2 : Double, y2 : Double) : Double {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
    }

    fun calculateRadian(x1 : Double, y1 : Double, x2 : Double, y2 : Double) : Double {
        return atan2(y2 - y1, x2 - x1)
    }

    fun radiansFromDegrees(degrees : Double) : Double {
        return degrees * (Math.PI / 180.0)
    }

    fun degreesFromRadians(radians : Double) : Double {
        return radians * (180.0 / Math.PI)
    }

    fun angleBetweenTwoPoints(x1 : Double, y1 : Double, x2 : Double, y2 : Double) : Double {
        return atan2(y2 - y1, x2 - x1)
    }
}