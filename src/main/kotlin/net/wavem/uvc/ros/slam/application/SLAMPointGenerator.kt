package net.wavem.uvc.ros.slam.application

import net.wavem.uvc.ros.slam.application.math.CommonMath
import net.wavem.uvc.ros.slam.application.math.GTSMath
import net.wavem.uvc.ros.slam.application.math.KTMMath
import net.wavem.uvc.ros.slam.application.math.SLAMMath
import net.wavem.uvc.ros.slam.domain.GTSPoint
import net.wavem.uvc.ros.slam.domain.GeoPoint
import net.wavem.uvc.ros.slam.domain.KTMPoint
import net.wavem.uvc.ros.slam.domain.SLAMPoint
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import kotlin.math.*

object SLAMPointGenerator {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    fun locationFromLocation(geoPoint : GeoPoint, distance : Double, bearingDegrees : Double) : GeoPoint {
        val distanceKm : Double = distance / 1000.0
        val distanceRadians : Double = distanceKm / 6371.0
        val bearingRadians : Double = CommonMath.radiansFromDegrees(bearingDegrees)
        val fromLatRadians : Double = CommonMath.radiansFromDegrees(geoPoint.latitude)
        val fromLonRadians : Double = CommonMath.radiansFromDegrees(geoPoint.longitude)

        val toLatRadians : Double = asin(sin(fromLatRadians) * cos(distanceRadians) + cos(fromLatRadians) * sin(distanceRadians) * cos(bearingRadians))

        var toLonRadians : Double = fromLonRadians + atan2(sin(bearingRadians) * sin(distanceRadians) * cos(fromLatRadians), cos(distanceRadians) - sin(fromLatRadians) * sin(toLatRadians))

        toLonRadians = ((toLonRadians + 3 * PI) % (2 * PI)) - PI

        return GeoPoint(
            CommonMath.degreesFromRadians(toLatRadians),
            CommonMath.degreesFromRadians(toLonRadians)
        )
    }

    fun geoPointToSlamPoint(gts : GTSPoint, ktmDes : KTMPoint) : SLAMPoint {
        val ktmBcm : KTMPoint = gts.ktmPoint1

        logger.info("gts slam distance ${SLAMMath.distanceFormula(gts.slamPoint1, gts.slamPoint2).toString()}")
        logger.info(
            "gts ktm distance" + CommonMath.distanceFormula(
                gts.ktmPoint1.x, gts.ktmPoint1.y,
                gts.ktmPoint2.x, gts.ktmPoint2.y
            ).toString()
        )

        val distance : Double  = CommonMath.distanceFormula(
            ktmBcm.x, ktmBcm.y,
            ktmDes.x, ktmDes.y
        )

        logger.info("distance $distance")

        val rotateAngle : Double  = GTSMath.calculateRotation(gts)

        val startKtmToDesKtmRadian : Double  = CommonMath.calculateRadian(
            ktmBcm.x, ktmBcm.y,
            ktmDes.x, ktmDes.y
        )

        logger.info("start ktm to des ktm radian ${startKtmToDesKtmRadian.toString()}")

        val radian : Double  = startKtmToDesKtmRadian + rotateAngle + PI

        val x : Double  = gts.slamPoint1.x + distance * cos(radian)
        val y : Double  = gts.slamPoint1.y + distance * sin(radian)

        logger.info("rotate angle ${CommonMath.degreesFromRadians(rotateAngle).toString()}")

        return SLAMPoint(x, y)
    }

    fun slamPointToGeoPoint(gts : GTSPoint, slamDes : SLAMPoint) : GeoPoint {
        val slamBcm : SLAMPoint = gts.slamPoint1
        val distance : Double  = SLAMMath.distanceFormula(slamBcm, slamDes)
        val rotateAngle : Double  = GTSMath.calculateRotation(gts)
        val startSlamToDesSlamRadian : Double  = CommonMath.calculateRadian(
            slamBcm.x, slamBcm.y,
            slamDes.x, slamDes.y
        )

        // 수정필요: 마이너스 플러스 조정 필요할 것으로 예상.
        val radian : Double  = startSlamToDesSlamRadian - rotateAngle - PI

        val x : Double  = gts.ktmPoint1.x + distance * cos(radian)
        val y : Double  = gts.ktmPoint1.y + distance * sin(radian)

        return KTMMath.ktmPointToGeoPoint(x, y)
    }
}