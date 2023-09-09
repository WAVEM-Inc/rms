package net.wavem.uvc.mqtt.infra

import com.fasterxml.jackson.databind.ObjectMapper
import net.wavem.uvc.mqtt.domain.MqttProperties
import org.eclipse.paho.client.mqttv3.MqttAsyncClient
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.integration.annotation.Gateway
import org.springframework.integration.annotation.MessagingGateway
import org.springframework.integration.dsl.Transformers
import org.springframework.integration.dsl.integrationFlow
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory
import org.springframework.integration.mqtt.core.MqttPahoClientFactory
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter
import org.springframework.integration.mqtt.support.MqttHeaders
import org.springframework.messaging.MessageHandler
import org.springframework.messaging.handler.annotation.Header

@Configuration
class MqttConfiguration(
    private val sampleMessageHandler: SampleMessageHandler,
    private val mqttProperties: MqttProperties,
    private val objectMapper: ObjectMapper,
) {

    @Bean
    fun mqttPahoClientFactory(): MqttPahoClientFactory {
        return DefaultMqttPahoClientFactory()
            .apply {
                connectionOptions = connectOptions()
            }
    }

    private fun connectOptions(): MqttConnectOptions {
        return MqttConnectOptions()
            .apply {
                serverURIs = arrayOf(mqttProperties.connectionInfo())
            }
    }

    @Bean
    fun sampleMessageInboundFlow() = integrationFlow(mqttChannelAdapter("/sample", 1)) {
        try {
            transform(Transformers.fromJson(SampleMessage::class.java))
            handle {
                sampleMessageHandler.handle(it.payload as SampleMessage)
            }
        } catch (e: MqttException) {
            println("mqtt sampleMessageInboundFlow error occurred ${e.message}")
        }
    }

    private fun mqttChannelAdapter(topic: String, qos: Int): MqttPahoMessageDrivenChannelAdapter {
        return MqttPahoMessageDrivenChannelAdapter(
            MqttClient.generateClientId(),
            mqttPahoClientFactory(),
            topic)
            .apply {
                setCompletionTimeout(5000)
                setConverter(DefaultPahoMessageConverter())
                setQos(qos)
            }
    }

    @Bean
    fun mqttOutboundFlow() = integrationFlow(MQTT_OUTBOUND_CHANNEL) {
        transform<Any> {
            when (it) {
                is SampleMessage -> objectMapper.writeValueAsString(it)
                else -> it
            }
        }
        handle(mqttOutboundMessageHandler())
    }

    private fun mqttOutboundMessageHandler(): MessageHandler { // (10)
        return MqttPahoMessageHandler(MqttAsyncClient.generateClientId(), mqttPahoClientFactory())
            .apply {
                setAsync(true)
                setDefaultTopic(mqttProperties.topic)
                setDefaultQos(mqttProperties.qos)
            }
    }

    @MessagingGateway(defaultRequestChannel = MQTT_OUTBOUND_CHANNEL)
    interface MqttOutboundGateway {

        @Gateway
        fun publish(@Header(MqttHeaders.TOPIC) topic: String, data: String)

        @Gateway
        fun publish(data: SampleMessage)
    }

    companion object {
        const val MQTT_OUTBOUND_CHANNEL = "outboundChannel"
    }
}