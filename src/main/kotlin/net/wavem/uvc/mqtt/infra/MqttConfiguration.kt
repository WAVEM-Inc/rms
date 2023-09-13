package net.wavem.uvc.mqtt.infra

import com.fasterxml.jackson.databind.ObjectMapper
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.domain.MqttProperties
import net.wavem.uvc.rms.domain.RmsCommonProperties
import net.wavem.uvc.rms.gateway.env_config.domain.EnvConfig
import net.wavem.uvc.rms.gateway.env_config.request.EnvConfigRequestHandler
import net.wavem.uvc.rms.gateway.event.domain.Event
import net.wavem.uvc.rms.gateway.event.gateway.EventRequestHandler
import net.wavem.uvc.rms.gateway.location.domain.Location
import net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.domain.CmdVelProperties
import net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.request.CmdVelRequestHandler
import net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.response.CmdVelResponseHandler
import net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.domain.RobotPoseProperties
import net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.response.RobotPoseResponseHandler
import net.wavem.uvc.ros.geometry_msgs.msg.Pose
import net.wavem.uvc.ros.geometry_msgs.msg.Twist
import net.wavem.uvc.ros.nav_msgs.gateway.map.domain.MapServerMapProperties
import net.wavem.uvc.ros.nav_msgs.gateway.map.request.MapServerMapRequestHandler
import net.wavem.uvc.ros.nav_msgs.gateway.map.response.MapServerMapResponseHandler
import net.wavem.uvc.ros.nav_msgs.gateway.odometry.domain.OdometryProperties
import net.wavem.uvc.ros.nav_msgs.gateway.odometry.response.OdometryResponseHandler
import net.wavem.uvc.ros.nav_msgs.msg.Odometry
import net.wavem.uvc.ros.nav_msgs.srv.GetMap
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import net.wavem.uvc.ros.std_msgs.gateway.chatter.request.ChatterRequestHandler
import net.wavem.uvc.ros.std_msgs.gateway.chatter.response.ChatterResponseHandler
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
    private val log: MqttLogger,
    private val mqttProperties: MqttProperties,
    private val rmsCommonProperties: RmsCommonProperties,
    private val cmdVelProperties: CmdVelProperties,
    private val robotPoseProperties: RobotPoseProperties,
    private val mapServerMapProperties: MapServerMapProperties,
    private val odometryProperties: OdometryProperties,
    private val chatterProperties: ChatterProperties,
    private val objectMapper: ObjectMapper,
    private val envConfigRequestHandler: EnvConfigRequestHandler,
    private val eventRequestHandler: EventRequestHandler,
    private val cmdVelRequestHandler: CmdVelRequestHandler,
    private val cmdVelResponseHandler: CmdVelResponseHandler,
    private val robotPoseResponseHandler: RobotPoseResponseHandler,
    private val mapServerMapRequestHandler: MapServerMapRequestHandler,
    private val mapServerMapResponseHandler: MapServerMapResponseHandler,
    private val odometryResponseHandler: OdometryResponseHandler,
    private val chatterRequestHandler: ChatterRequestHandler,
    private val chatterResponseHandler: ChatterResponseHandler
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
            topic
        ).apply {
                setCompletionTimeout(5000)
                setConverter(DefaultPahoMessageConverter())
                setQos(qos)
            }
    }

    @Bean
    fun envConfigRequestToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        "/test/config",
        0
    )) {
        try {
            transform(Transformers.fromJson(EnvConfig::class.java))
            handle {
                envConfigRequestHandler.handle(it.payload as EnvConfig)
            }
        } catch (e: MqttException) {

        }
    }

    @Bean
    fun eventReqeustToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        "hubilon/atcplus/rms/event",
        0
    )) {
        try {
            transform(Transformers.fromJson(Event::class.java))
            handle {
                eventRequestHandler.handle(it.payload as Event)
            }
        } catch (e: MqttException) {

        }
    }

    @Bean
    fun cmdVelRequestToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        cmdVelProperties.toBridge,
        cmdVelProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(Twist::class.java))
            handle {
                cmdVelRequestHandler.handle(it.payload as Twist)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.TO_BRIDGE, "mqtt cmdVelRequestToBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun cmdVelResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        cmdVelProperties.fromBridge,
        cmdVelProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(Twist::class.java))
            handle {
                cmdVelResponseHandler.handle(it.payload as Twist)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.FROM_BRIDGE, "mqtt cmdVelResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun robotPoseResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        robotPoseProperties.fromBridge,
        robotPoseProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(Pose::class.java))
            handle {
                robotPoseResponseHandler.handle(it.payload as Pose)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.FROM_BRIDGE, "mqtt robotPoseResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun mapServerMapRequestToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        mapServerMapProperties.toBridge,
        mapServerMapProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(GetMap::class.java))
            handle {
                mapServerMapRequestHandler.handle(it.payload as GetMap)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.TO_BRIDGE, "mqtt mapServerMapRequestToBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun mapServerMapResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        mapServerMapProperties.fromBridge,
        mapServerMapProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(GetMap::class.java))
            handle {
                mapServerMapResponseHandler.handle(it.payload as GetMap)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.FROM_BRIDGE, "mqtt mapServerMapResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun odometryResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        odometryProperties.fromBridge,
        odometryProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(Odometry::class.java))
            handle {
                odometryResponseHandler.handle(it.payload as Odometry)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.FROM_BRIDGE, "mqtt odometryResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun chatterRequestToBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        chatterProperties.toBridge,
        chatterProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(net.wavem.uvc.ros.std_msgs.msg.String::class.java))
            handle {
                chatterRequestHandler.handle(it.payload as net.wavem.uvc.ros.std_msgs.msg.String)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.TO_BRIDGE, "mqtt chatterRequestToBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun chatterResponseFromBridge(): StandardIntegrationFlow = integrationFlow(mqttChannelAdapter(
        chatterProperties.fromBridge,
        chatterProperties.qos
    )) {
        try {
            transform(Transformers.fromJson(net.wavem.uvc.ros.std_msgs.msg.String::class.java))
            handle {
                chatterResponseHandler.handle(it.payload as net.wavem.uvc.ros.std_msgs.msg.String)
            }
        } catch (e: MqttException) {
            log.error(MqttConnectionType.FROM_BRIDGE, "mqtt chatterResponseFromBridge error occurred ${e.message}")
        }
    }

    @Bean
    fun mqttOutboundFlow(): StandardIntegrationFlow = integrationFlow(MQTT_OUTBOUND_CHANNEL) {
        transform<Any> {
            when (it) {
                is net.wavem.uvc.ros.std_msgs.msg.String -> objectMapper.writeValueAsString(it)
                is Location -> objectMapper.writeValueAsString(it)
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
    interface MqttOutboundGateway<T> {
        @Gateway
        fun publish(@Header(MqttHeaders.TOPIC) topic: String, data: T)

        @Gateway
        fun publish(@Header(MqttHeaders.TOPIC) topic: String, qos: Int, data: T)
    }

    companion object {
        const val MQTT_OUTBOUND_CHANNEL: String = "outboundChannel"
    }
}