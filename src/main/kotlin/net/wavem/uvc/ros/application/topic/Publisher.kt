package net.wavem.uvc.ros.application.topic

import id.jrosmessages.Message
import net.wavem.uvc.ros.infra.DDSQoS
import net.wavem.uvc.ros.infra.DDSSupport
import org.springframework.stereotype.Service
import pinorobotics.rtpstalk.RtpsTalkClient
import pinorobotics.rtpstalk.RtpsTalkConfiguration
import pinorobotics.rtpstalk.messages.RtpsTalkDataMessage
import java.util.concurrent.SubmissionPublisher
import kotlin.reflect.KClass

@Service
class Publisher<M : Message> {
    val ddsClient : RtpsTalkClient = RtpsTalkClient(
        RtpsTalkConfiguration.Builder()
            .networkInterface("lo")
            .build()
    )

    val ddsSupport : DDSSupport = DDSSupport()
    val ddsPublisher : SubmissionPublisher<RtpsTalkDataMessage> = SubmissionPublisher<RtpsTalkDataMessage>()

    fun registerPublisher(topic : String, message : KClass<M>) {
        val ddsTopic : String = ddsSupport.qualifyTopic(topic)
        val ddsMessageType : String = ddsSupport.qualifyMessageType(message)
        ddsClient.publish(ddsTopic, ddsMessageType, DDSQoS.DEFAULT_PUBLISHER_QOS, ddsPublisher)
    }

    fun publish(data : ByteArray) {
        val ddsMessage : RtpsTalkDataMessage = RtpsTalkDataMessage(data)
        ddsPublisher.submit(ddsMessage)
    }
}