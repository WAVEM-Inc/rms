package net.wavem.uvc.mqtt.infra

import org.springframework.stereotype.Component

@Component
class SampleMessageHandler {

    val list: MutableList<SampleMessage> = mutableListOf()

    fun handle(message: SampleMessage) {
        println("message arrived : $message")

        list.add(message)

        for (msg in list) {
            println("global message list message : $msg")
        }
    }
}

data class SampleMessage(val title: String, val content: String)