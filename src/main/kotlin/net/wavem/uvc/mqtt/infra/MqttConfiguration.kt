package net.wavem.uvc.mqtt.infra

import com.fasterxml.jackson.databind.ObjectMapper
import net.wavem.uvc.mqtt.domain.MqttProperties
import net.wavem.uvc.ros2.nav_msgs.srv.GetMap
import net.wavem.uvc.ros2.std_msgs.api.chatter.request.ChatterRequestHandler
import net.wavem.uvc.ros2.std_msgs.api.chatter.response.ChatterResponseHandler
import org.eclipse.paho.client.mqttv3.MqttAsyncClient
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration
import org.springframework.integration.annotation.Gateway
import org.springframework.integration.annotation.MessagingGateway
import org.springframework.integration.dsl.StandardIntegrationFlow
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
    private val chatterRequestHandler: ChatterRequestHandler,
    private val chatterResponseHandler: ChatterResponseHandler,
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
    fun chatterRequestToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter("/atc/uvc/request/chatter", 1)) {
        try {
            transform(Transformers.fromJson(net.wavem.uvc.ros2.std_msgs.msg.String::class.java))
            handle {
                chatterRequestHandler.handle(it.payload as net.wavem.uvc.ros2.std_msgs.msg.String)
            }
        } catch (e: MqttException) {
            println("mqtt chatterRequestToBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun chatterResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter("/response/chatter", 1)) {
        try {
            transform(Transformers.fromJson(net.wavem.uvc.ros2.std_msgs.msg.String::class.java))
            handle {
                chatterResponseHandler.handle(it.payload as net.wavem.uvc.ros2.std_msgs.msg.String)
            }
        } catch (e: MqttException) {
            println("mqtt chatterResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun mqttOutboundFlow(): StandardIntegrationFlow = integrationFlow(MQTT_OUTBOUND_CHANNEL) {
        transform<Any> {
            when (it) {
                is net.wavem.uvc.ros2.std_msgs.msg.String -> objectMapper.writeValueAsString(it)
                is GetMap -> objectMapper.writeValueAsString(it)
                else -> it
            }
        }
        handle(mqttOutboundMessageHandler())
    }

    private fun mqttOutboundMessageHandler(): MessageHandler {
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
        fun publish(@Header(MqttHeaders.TOPIC) topic: String, data: net.wavem.uvc.ros2.std_msgs.msg.String)

        @Gateway
        fun publish(@Header(MqttHeaders.TOPIC) topic: String, data: GetMap)
    }

    companion object {
        const val MQTT_OUTBOUND_CHANNEL: String = "outboundChannel"
    }
}