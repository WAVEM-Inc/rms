package net.wavem.uvc.ros.domain.sensor_msgs

import id.jrosmessages.Message
import id.xfunction.XJson
import net.wavem.uvc.ros.domain.std_msgs.Header
import net.wavem.uvc.ros.domain.builtin_interfaces.Time
import java.nio.ByteBuffer
import java.nio.ByteOrder

class NavSatFix() : Message {

    var header : Header = Header()
    var status : NavSatStatus = NavSatStatus()
    var latitude : Double = 0.0
    var longitude : Double = 0.0
    var altitude : Double = 0.0
    var position_covariance : DoubleArray = DoubleArray(9)
    var position_covariance_type : UByte  = 0u

    constructor(
        header : Header,
        status : NavSatStatus,
        latitude : Double,
        longitude : Double,
        altitude : Double,
        position_covariance : DoubleArray,
        position_covariance_type : UByte
    ) : this() {
        this.header = header
        this.status = status
        this.latitude = latitude
        this.longitude = longitude
        this.altitude = altitude
        this.position_covariance = position_covariance
        this.position_covariance_type = position_covariance_type
    }

    fun write() : ByteArray {
        val buf : ByteBuffer = ByteBuffer.allocate(16 + (Byte.SIZE_BYTES + UShort.SIZE_BYTES) + (Double.SIZE_BYTES * 3) + Int.SIZE_BYTES + (Double.SIZE_BYTES * 9) + UByte.SIZE_BYTES)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        val headerBytes : ByteArray = this.header.write()
        buf.put(headerBytes)

        val navSatStatusBytes : ByteArray = this.status.write()
        buf.put(navSatStatusBytes)

        buf.putDouble(this.latitude)
        buf.putDouble(this.longitude)
        buf.putDouble(this.altitude)

        buf.putInt(Double.SIZE_BYTES * 9)

        for (covariance in position_covariance) {
            buf.putDouble(covariance)
        }

        buf.put(this.position_covariance_type.toByte())

        return buf.array()
    }

    override fun equals(other : Any?) : Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as NavSatFix

        return position_covariance.contentEquals(other.position_covariance)
    }

    override fun hashCode() : Int {
        return position_covariance.contentHashCode()
    }

    override fun toString() : String {
        return XJson.asString(
            "header", this.header.toString(),
            "status", this.status.toString(),
            "latitude", this.latitude,
            "longitude", this.longitude,
            "altitude", this.altitude,
            "position_covariance", this.position_covariance,
            "position_covariance_type", this.position_covariance_type
        )
    }

    companion object {
        const val COVARIANCE_TYPE_UNKNOWN : Byte = 0
        const val COVARIANCE_TYPE_APPROXIMATED : Byte = 1
        const val COVARIANCE_TYPE_DIAGONAL_KNOWN : Byte = 2
        const val COVARIANCE_TYPE_KNOWN : Byte = 3

        fun read(data : ByteArray) : NavSatFix {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)

            val header : Header = Header.read(data)
            val headerBufferSize : Int = Header.getBufferSize()

            buf.position(headerBufferSize)

            val status : NavSatStatus = NavSatStatus.read(data, headerBufferSize)

            buf.position(headerBufferSize + Double.SIZE_BYTES)

            val latitude : Double = buf.getDouble()

            val longitude : Double = buf.getDouble()

            val altitude : Double = buf.getDouble()

            val position_covariance_size : Int = 9

            val position_covariance : DoubleArray = DoubleArray(position_covariance_size)

            for (i in 0 .. position_covariance_size - 1) {
                val covariance : Double = buf.getDouble()
                position_covariance[i] = covariance
            }
            
            val position_covariance_type : UByte = buf.get().toUByte()

            return NavSatFix(
                header = header,
                status = status,
                latitude = latitude,
                longitude = longitude,
                altitude = altitude,
                position_covariance = position_covariance,
                position_covariance_type = position_covariance_type
            )
        }
    }
}