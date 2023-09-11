package net.wavem.uvc.mqtt.infra

interface MqttHandler<T> {
    fun handle(data: T)
}