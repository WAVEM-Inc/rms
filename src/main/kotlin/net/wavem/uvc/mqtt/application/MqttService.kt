package net.wavem.uvc.mqtt.application

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.core.io.ClassPathResource
import org.springframework.stereotype.Service
import java.io.InputStream

@Service
class MqttService<T>(
    private val mqttLogger : MqttLogger,
    private val mqttOutboundGateway : MqttConfiguration.MqttOutboundGateway<T>
) {

    fun readMQTTYML() : JsonObject {
        try {
            val configFile : ClassPathResource = ClassPathResource("mqtt.json")
            val inputStream : InputStream = configFile.inputStream
            val jsonText : String = inputStream.bufferedReader().use { it.readText() }

            return Gson().fromJson(jsonText, JsonObject::class.java)
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("Failed to load MQTT config file", e)
        }
    }

    fun buildMQTTURLFromYML() : String {
        val mqttConnectionInfoJson : JsonObject = this.readMQTTYML()

        return "${mqttConnectionInfoJson.get("protocol").asString}://${mqttConnectionInfoJson.get("ip").asString}:${mqttConnectionInfoJson.get("port")}"
    }

    fun processROSData(connectionType : MqttConnectionType, topic : String, data : T) : T {
        mqttLogger.info(connectionType, "[$topic] message arrived : [$data]")

        return data
    }

    fun bridge(connectionType : MqttConnectionType, topic : String, data : T) {
        mqttOutboundGateway.publish(topic = topic, data = data)
    }
}