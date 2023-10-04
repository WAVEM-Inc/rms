package net.wavem.uvc.ros.slam.domain

data class GeoPoint(var latitude : Double, var longitude : Double) {
    override fun toString() : String {
        return "GEOPoint latitude -> $latitude, longitude -> $longitude"
    }
}