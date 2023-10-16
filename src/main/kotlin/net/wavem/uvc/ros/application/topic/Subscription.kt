package net.wavem.uvc.ros.application.topic

import id.jrosmessages.Message
import net.wavem.uvc.ros.infra.DDSQoS
import net.wavem.uvc.ros.infra.DDSSupport
import org.springframework.stereotype.Service
import pinorobotics.rtpstalk.RtpsTalkClient
import pinorobotics.rtpstalk.RtpsTalkConfiguration
import pinorobotics.rtpstalk.messages.RtpsTalkDataMessage
import rx.Observable
import rx.subjects.PublishSubject
import java.util.concurrent.Flow
import kotlin.reflect.KClass

@Service
class Subscription<M : Message> {
    val ddsClient : RtpsTalkClient = RtpsTalkClient(
        RtpsTalkConfiguration.Builder()
            .networkInterface("lo")
            .build()
    )

    val ddsSupport : DDSSupport = DDSSupport()
    val dataObservable : PublishSubject<ByteArray> = PublishSubject.create()

    fun getDataObservable() : Observable<ByteArray> {
        return dataObservable
    }

    fun registerSubscription(topic : String, message : KClass<M>) {
        val ddsTopic : String = ddsSupport.qualifyTopic(topic)
        val ddsMessageType : String = ddsSupport.qualifyMessageType(message)

        ddsClient.subscribe(ddsTopic, ddsMessageType, DDSQoS.DEFAULT_SUBSCRIBER_QOS, object : Flow.Subscriber<RtpsTalkDataMessage> {
            private lateinit var subscription : Flow.Subscription

            override fun onSubscribe(subscription : Flow.Subscription) {
                this.subscription = subscription
                println("$ddsTopic subscription registered")
                subscription.request(1)
            }

            override fun onNext(message : RtpsTalkDataMessage) {
                message.data().ifPresent { data ->
                    dataObservable.onNext(data)
                }
                subscription.request(1)
            }

            override fun onError(throwable : Throwable) {
                throwable.printStackTrace()
            }

            override fun onComplete() {
                subscription.cancel()
            }
        })
    }
}