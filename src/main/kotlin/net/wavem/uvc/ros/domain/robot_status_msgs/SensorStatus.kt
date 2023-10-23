package net.wavem.uvc.ros.domain.robot_status_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.std_msgs.Header
import java.nio.ByteBuffer
import java.nio.ByteOrder

class SensorStatus() : Message {
    var header : Header = Header()
    var status_code : Int = 0
    var status_message : String = ""

    constructor(header : Header, status_code : Int, status_message : String) : this() {
        this.header = header
        this.status_code = status_code
        this.status_message = status_message
    }

    companion object {
        fun read(data : ByteArray) : SensorStatus {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)

            val header : Header = Header.read(data)
            val headerBufferSize : Int = Header.getBufferSize()

            buf.position(headerBufferSize)

            val status_code : Int = buf.getInt()

            var status_message_len : Int = buf.getInt()
            var status_message : String = ""

            while (status_message_len-- > 0) status_message += Char(buf.get().toUShort())

            return SensorStatus(
                header = header,
                status_code = status_code,
                status_message = status_message
            )
        }
    }
}