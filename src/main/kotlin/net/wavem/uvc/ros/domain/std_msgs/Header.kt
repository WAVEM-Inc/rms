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
        val buf : ByteBuffer = ByteBuffer.allocate(Integer.BYTES * 2 + frameIdLen)
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

        var gLen : Int = 0
        var gFrameIdLen : Int = 0

        fun getBufferSize() : Int {
            return (Time.getBufferSize() + gLen + gFrameIdLen)
        }

        fun read(data : ByteArray, position : Int = 0) : Header {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)
            buf.position(position)

            val time : Time = Time.read(data)

            val timeSize : Int = Time.getBufferSize()

            buf.position(timeSize)

            var len = buf.getInt()
            gLen = len

            var frame_id : kotlin.String = ""
            
            while (len-- > 0) frame_id += Char(buf.get().toUShort())

            gFrameIdLen = frame_id.length

            return Header(
                stamp = time,
                frame_id = frame_id
            )
        }
    }
}