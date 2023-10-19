package net.wavem.uvc.ros.domain.builtin_interfaces

import id.jrosmessages.Message
import id.xfunction.XJson
import java.nio.ByteBuffer
import java.nio.ByteOrder

class Time() : Message {
    private var sec : Int = 0
    private var nanosec : Int = 0

    constructor(sec : Int, nanosec : Int) : this() {
        this.sec = sec
        this.nanosec = nanosec
    }

    fun getSize() : Int {
        return (Int.SIZE_BYTES * 2)
    }

    fun write() : ByteArray {
        val buf : ByteBuffer = ByteBuffer.allocate(this.getSize())
        buf.order(ByteOrder.LITTLE_ENDIAN)

        buf.putInt(this.sec)
        buf.putInt(this.nanosec)

        return buf.array()
    }

    override fun toString() : String {
        return XJson.asString(
            "sec", this.sec,
            "nanosec", this.nanosec
        )
    }

    companion object {

        fun getBufferSize() : Int {
            return (Int.SIZE_BYTES * 2)
        }

        fun read(data : ByteArray) : Time {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)

            val sec : Int = buf.getInt()
            val nanosec : Int = buf.getInt()

            return Time(
                sec = sec,
                nanosec = nanosec
            )
        }
    }
}