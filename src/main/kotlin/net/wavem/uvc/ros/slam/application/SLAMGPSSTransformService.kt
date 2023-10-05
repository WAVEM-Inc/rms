package net.wavem.uvc.ros.slam.application

import net.wavem.uvc.ros.slam.application.math.CommonMath
import net.wavem.uvc.ros.slam.application.math.GPSMath
import net.wavem.uvc.ros.slam.application.math.KTMMath
import net.wavem.uvc.ros.slam.domain.GTSPoint
import net.wavem.uvc.ros.slam.domain.GeoPoint
import net.wavem.uvc.ros.slam.domain.KTMPoint
import net.wavem.uvc.ros.slam.domain.SLAMPoint
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Service

@Service
class SLAMGPSSTransformService {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    fun transform() {
        val startKTMPoint : KTMPoint = KTMMath.geoPointToKtmPoint(37.465932, 127.124136)
        val endKTMPoint : KTMPoint = KTMMath.geoPointToKtmPoint(37.465798, 127.124107)

        logger.info("start $startKTMPoint")
        logger.info("end $endKTMPoint")

        val startSLAMPoint : SLAMPoint = SLAMPoint(2.724780, -2.330876)
        val endSLAMPoint : SLAMPoint = SLAMPoint(-12.386119, 0.497768)

        val gtsPoint : GTSPoint = GTSPoint(startKTMPoint, startSLAMPoint, endKTMPoint, endSLAMPoint)

        val des1 : KTMPoint = KTMMath.geoPointToKtmPoint(37.4658517, 127.1239257)
        val des2 : KTMPoint = KTMMath.geoPointToKtmPoint(37.465798, 127.124107)

        val currentKTMPoint : KTMPoint = KTMMath.geoPointToKtmPoint(37.4658517, 127.1239257) // 현재 UGV 위치 - GPS
        val result : SLAMPoint = SLAMPointGenerator.geoPointToSlamPoint(gtsPoint, currentKTMPoint) // 현재 UGV 위치 - 계산된 SLAM 좌표

        initialPose(result)

        val geoPoint1 : GeoPoint = GeoPoint(37.465932, 127.124136)
        val geoPoint2 : GeoPoint = GeoPoint(37.465798, 127.124107)

        logger.info("StartPoint X: ${startKTMPoint.x}, Y: ${startKTMPoint.y}")
        logger.info("EndPoint X: ${endKTMPoint.x}, Y: ${endKTMPoint.y}")
        logger.info("differ X: ${startKTMPoint.x - endKTMPoint.x}, Y: ${startKTMPoint.y - endKTMPoint.y}")
        logger.info("GPS Distance ${GPSMath.distanceFormulaMeters(geoPoint1, geoPoint2)}")
        logger.info("KTM Distance ${
            CommonMath.distanceFormula(
                startKTMPoint.x,
                startKTMPoint.y,
                endKTMPoint.x,
                endKTMPoint.y
            )
        }")
        logger.info("계산 ${CommonMath.distanceFormula(0.0, 0.0, 2.5459084357135, 14.875566659029573)}")

        val resolution : Double = 0.05
        val maxKTMPoint : KTMPoint = KTMMath.geoPointToKtmPoint(37.465932, 127.124136)
        val minKTMPoint : KTMPoint = KTMMath.geoPointToKtmPoint(37.465798, 127.124107)

        val width : Int = ((maxKTMPoint.x - minKTMPoint.x) / resolution).toInt()
        val height : Int = ((maxKTMPoint.y - minKTMPoint.y) / resolution).toInt()
    }

    fun initialPose(slamPoint : SLAMPoint) {

    }
}