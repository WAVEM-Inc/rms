package net.wavem.uvc.ros.slam.domain

class GTSPoint(
    var ktmPoint1 : KTMPoint,
    var slamPoint1 : SLAMPoint,
    var ktmPoint2 : KTMPoint,
    var slamPoint2 : SLAMPoint
) {
    override fun toString() : String {
        return "GTSPoint{" +
                "ktmPoint1=" + ktmPoint1 +
                ", slamPoint1=" + slamPoint1 +
                ", ktmPoint2=" + ktmPoint2 +
                ", slamPoint2=" + slamPoint2 +
                '}'
    }
}
