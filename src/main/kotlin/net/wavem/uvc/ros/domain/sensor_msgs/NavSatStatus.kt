package net.wavem.uvc.ros.domain.sensor_msgs

import id.jrosmessages.Message
import id.xfunction.XJson
import java.nio.ByteBuffer
import java.nio.ByteOrder

class NavSatStatus() : Message {
    var status : Byte = 0
    var service : UShort = 0u

    constructor(status : Byte, service : UShort) : this() {
        this.status = status
        this.service = service
    }

    override fun toString() : String {
        return XJson.asString(
            "status", this.status,
            "service", this.service
        )
    }

    companion object {
        const val STATUS_NO_FIX : Byte = -1
        const val STATUS_FIX : Byte = 0
        const val STATUS_SBAS_FIX : Byte = 1
        const val STATUS_GBAS_FIX : Byte = 2
        const val SERVICE_GPS : UShort = 1u
        const val SERVICE_GLONASS : UShort = 2u
        const val SERVICE_COMPASS : UShort = 4u
        const val SERVICE_GALILEO : UShort = 8u

        fun getBufferSize() : Int {
            return (Byte.SIZE_BYTES + UShort.SIZE_BYTES)
        }

        fun read(data : ByteArray, position : Int = 0) : NavSatStatus {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)
            buf.position(position)

            val status : Byte = buf.get()

            buf.position(position + 2)
            
            val service : UShort = buf.getShort().toUShort()

            return NavSatStatus(
                status = status,
                service = service
            )
        }
    }
}