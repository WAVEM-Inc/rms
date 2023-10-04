package net.wavem.uvc.ros.slam.application.math

import net.wavem.uvc.ros.slam.domain.GeoPoint
import net.wavem.uvc.ros.slam.domain.KTMPoint
import org.locationtech.proj4j.*

object KTMMath {
    private val crsFactory : CRSFactory = CRSFactory()
    private val wgs84 : CoordinateReferenceSystem = crsFactory.createFromName("epsg:4326")
    private val ktm : CoordinateReferenceSystem = crsFactory.createFromName("epsg:5186")

    fun geoPointToKtmPoint(latitude : Double, longitude : Double) : KTMPoint {
        val ctFactory : CoordinateTransformFactory = CoordinateTransformFactory()
        val wgsToKtm : CoordinateTransform = ctFactory.createTransform(wgs84, ktm)

        val result : ProjCoordinate = ProjCoordinate()
        wgsToKtm.transform(ProjCoordinate(longitude, latitude), result)

        val ktmX : Double = result.x
        val ktmY : Double = result.y

        return KTMPoint(ktmX, ktmY)
    }

    fun ktmPointToGeoPoint(x : Double, y : Double) : GeoPoint {
        val ctFactory : CoordinateTransformFactory = CoordinateTransformFactory()
        val ktmToWgs : CoordinateTransform = ctFactory.createTransform(ktm, wgs84)

        val result : ProjCoordinate = ProjCoordinate()
        ktmToWgs.transform(ProjCoordinate(x, y), result)

        val lat : Double = result.y
        val lon : Double = result.x

        return GeoPoint(lat, lon)
    }
}
