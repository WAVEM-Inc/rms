package net.wavem.uvc.ros.slam.domain

data class SLAMPoint(var x : Double, var y : Double) {
    override fun toString() : String {
        return "SLAMPoint{" +
                "x=$x, y=$y" +
                '}'
    }
}
