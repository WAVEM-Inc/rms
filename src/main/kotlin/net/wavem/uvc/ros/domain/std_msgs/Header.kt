package net.wavem.uvc.ros.domain.std_msgs

import id.jrosmessages.Message
import id.xfunction.XJson
import net.wavem.uvc.ros.domain.builtin_interfaces.Time
import java.nio.ByteBuffer
import java.nio.ByteOrder


class Header() : Message {
    var stamp : Time = Time()
    var frame_id : kotlin.String = ""

    constructor(stamp : Time, frame_id : kotlin.String) : this() {
        this.stamp = stamp
        this.frame_id = frame_id
    }

    fun write() : ByteArray {
        val frameIdLen : Int = this.frame_id.length + 1
        val buf : ByteBuffer = ByteBuffer.allocate(Integer.BYTES * 2)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        val timeBytes : ByteArray = this.stamp.write()
        buf.put(timeBytes)

        buf.putInt(frameIdLen)
        buf.put(this.frame_id.toByteArray())

        return buf.array()
    }

    override fun toString() : String {
        return XJson.asString(
            "stamp", this.stamp,
            "frame_id", this.frame_id
        )
    }

    companion object {

        var len : Int = 0

        fun getBufferSize() : Int {
            return (Time.getBufferSize() + len)
        }

        fun read(data : ByteArray) : Header {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)

            val timeSize : Int = Time.getBufferSize()
            println("Header timeSize : $timeSize")
            val time : Time = Time.read(data)
            println("Header time : $time")
            buf.position(timeSize)

            len = buf.getInt()
            var frame_id : kotlin.String = ""

            while (len-- > 0) frame_id += Char(buf.get().toUShort())

            return Header(
                stamp = time,
                frame_id = frame_id
            )
        }
    }
}