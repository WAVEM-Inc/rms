package net.wavem.uvc.mqtt.infra

import com.fasterxml.jackson.databind.ObjectMapper
import lombok.RequiredArgsConstructor
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.gateway.control.domain.Control
import net.wavem.uvc.rms.gateway.control.request.ControlRequestHandler
import net.wavem.uvc.rms.gateway.env_config.domain.EnvConfig
import net.wavem.uvc.rms.gateway.env_config.request.EnvConfigRequestHandler
import net.wavem.uvc.rms.gateway.event.domain.Event
import net.wavem.uvc.rms.gateway.event.response.EventResponseHandler
import net.wavem.uvc.rms.gateway.location.domain.Location
import net.wavem.uvc.rms.gateway.path.domain.Path
import net.wavem.uvc.rms.gateway.path.request.PathRequestHandler
import org.eclipse.paho.client.mqttv3.MqttAsyncClient
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttException
import org.slf4j.Logger
import org.slf4j.LoggerFactory
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
@RequiredArgsConstructor
class MqttConfiguration(
    private val log : MqttLogger,
    private val mqttService : MqttService<Any>,
    private val objectMapper : ObjectMapper,
    private val pathRequestHandler : PathRequestHandler,
    private val controlRequestHandler : ControlRequestHandler,
    private val envConfigRequestHandler : EnvConfigRequestHandler,
) {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    @Bean
    fun mqttPahoClientFactory() : MqttPahoClientFactory {
        return DefaultMqttPahoClientFactory()
            .apply {
                connectionOptions = connectOptions()
            }
    }

    private fun connectOptions() : MqttConnectOptions {
        val mqttUrl: String = mqttService.buildMQTTURLFromYML()

        return MqttConnectOptions()
            .apply {
                logger.info("======== MQTT Connected to [$mqttUrl] ========")
                serverURIs = arrayOf(mqttUrl)
            }
    }

    private fun mqttChannelAdapter(topic : String, qos : Int) : MqttPahoMessageDrivenChannelAdapter {
        return MqttPahoMessageDrivenChannelAdapter(
            MqttClient.generateClientId(),
            mqttPahoClientFactory(),
            topic
        ).apply {
                setCompletionTimeout(5000)
                setConverter(DefaultPahoMessageConverter())
                setQos(qos)
            }
    }

    @Bean
    fun pathRequestToBridge() : StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        "hubilon/atcplus/rms/path",
        2,
    )) {
        try {
            transform(Transformers.fromJson(Path::class.java))
            handle {
                pathRequestHandler.handle(it.payload as Path)
            }
        } catch (e : MqttException) {
            log.error(MqttConnectionType.TO_RMS, "mqtt pathRequestToBridge error occurred ${e.message}")
            e.printStackTrace()
        }
    }

    @Bean
    fun controlRequestToBridge() : StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        "hubilon/atcplus/rms/control",
        1
    )) {
        try {
            transform(Transformers.fromJson(Control::class.java))
            handle {
                controlRequestHandler.handle(it.payload as Control)
            }
        } catch (e : MqttException) {
            log.error(MqttConnectionType.TO_RMS, "mqtt controlRequestToBridge error occurred ${e.message}")
            e.printStackTrace()
        }
    }

    @Bean
    fun envConfigRequestToBridge() : StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        "hubilon/atcplus/rms/config",
        0
    )) {
        try {
            transform(Transformers.fromJson(EnvConfig::class.java))
            handle {
                envConfigRequestHandler.handle(it.payload as EnvConfig)
            }
        } catch (e : MqttException) {
            log.error(MqttConnectionType.TO_RMS, "mqtt envConfigRequestToBridge error occurred ${e.message}")
            e.printStackTrace()
        }
    }

    @Bean
    fun mqttOutboundFlow() : StandardIntegrationFlow = integrationFlow(MQTT_OUTBOUND_CHANNEL) {
        transform<Any> {
            when (it) {
                is Location -> objectMapper.writeValueAsString(it)
                is Event -> objectMapper.writeValueAsString(it)
                else -> it
            }
        }
        handle(mqttOutboundMessageHandler())
    }

    private fun mqttOutboundMessageHandler() : MessageHandler {
        return MqttPahoMessageHandler(MqttAsyncClient.generateClientId(), mqttPahoClientFactory())
            .apply {
                setAsync(true)
            }
    }

    @MessagingGateway(defaultRequestChannel = MQTT_OUTBOUND_CHANNEL)
    interface MqttOutboundGateway<T> {
        @Gateway
        fun publish(@Header(MqttHeaders.TOPIC) topic : String, data : T)
    }

    private companion object {
        const val MQTT_OUTBOUND_CHANNEL : String = "outboundChannel"
    }
}