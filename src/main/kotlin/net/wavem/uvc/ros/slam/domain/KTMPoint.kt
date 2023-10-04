package net.wavem.uvc.ros.slam.domain

data class KTMPoint(var x : Double, var y : Double) {
    override fun toString() : String {
        return "KTMPoint{" +
                "x=$x, y=$y" +
                '}'
    }
}
