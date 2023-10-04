package net.wavem.uvc.ros.slam.application.math

import net.wavem.uvc.ros.slam.domain.GeoPoint
import kotlin.math.*

object GPSMath {
    const val EARTH_RADIUS_KM : Double = 6371.0
    const val EARTH_RADIUS_METER : Double = EARTH_RADIUS_KM * 1000

    private fun distanceFormulaKilometers(lat1 : Double, lon1 : Double, lat2 : Double, lon2 : Double) : Double {
        val phi1 : Double = Math.toRadians(lat1)
        val phi2 : Double = Math.toRadians(lat2)
        val deltaPhi : Double = Math.toRadians(lat2 - lat1)
        val deltaLambda : Double = Math.toRadians(lon2 - lon1)

        val a : Double = sin(deltaPhi / 2.0).pow(2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2.0).pow(2)
        val c : Double = 2 * atan2(sqrt(a), sqrt(1 - a))

        return EARTH_RADIUS_KM * c
    }

    fun distanceFormulaKilometers(startPoint : GeoPoint, endPoint : GeoPoint) : Double {
        return distanceFormulaKilometers(
            startPoint.latitude,
            startPoint.longitude,
            endPoint.latitude,
            endPoint.longitude
        )
    }

    private fun distanceFormulaMeters(lat1 : Double, lon1 : Double, lat2 : Double, lon2 : Double) : Double {
        return distanceFormulaKilometers(lat1, lon1, lat2, lon2) * 1000
    }

    fun distanceFormulaMeters(startPoint : GeoPoint, endPoint : GeoPoint) : Double {
        return distanceFormulaMeters(
            startPoint.latitude,
            startPoint.longitude,
            endPoint.latitude,
            endPoint.longitude
        )
    }

    private fun angleBetweenTwoPoints(lat1 : Double, lon1 : Double, lat2 : Double, lon2 : Double) : Double {
        val phi1 : Double = lat1 * Math.PI / 180
        val phi2 : Double = lat2 * Math.PI / 180
        val lambda1 : Double = lon1 * Math.PI / 180
        val lambda2 : Double = lon2 * Math.PI / 180

        val y : Double = sin(lambda2 - lambda1) * cos(phi2)
        val x : Double = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(lambda2 - lambda1)

        return atan2(y, x)
    }

    fun angleBetweenTwoPoints(startPoint : GeoPoint, endPoint : GeoPoint) : Double {
        return angleBetweenTwoPoints(
            startPoint.latitude,
            startPoint.longitude,
            endPoint.latitude,
            endPoint.longitude
        )
    }
}