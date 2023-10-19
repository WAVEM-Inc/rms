package net.wavem.uvc.rms.gateway.env_config.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MQTTService
import net.wavem.uvc.mqtt.domain.MQTTConnectionType
import net.wavem.uvc.mqtt.infra.MQTTLogger
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.env_config.domain.EnvConfig
import net.wavem.uvc.rms.gateway.env_config.domain.EnvConfigProperties
import net.wavem.uvc.rms.gateway.env_config.domain.set_info.EnvConfigSetInfo
import org.springframework.core.io.ClassPathResource
import org.springframework.stereotype.Component
import java.io.FileWriter
import java.io.InputStream

@Component
class EnvConfigRequestHandler(
    val log : MQTTLogger,
    val envConfigProperties : EnvConfigProperties,
    val mqttService : MQTTService<String>,
    val gson : Gson
) {

    private fun renewMQTTIP(setInfoJson : JsonObject) {
        val previousMqttConfigFile : ClassPathResource = ClassPathResource("mqtt.json")
        val previousMqttConfigFileInputStream : InputStream = previousMqttConfigFile.inputStream
        val previousMqttConfigFileText : String = previousMqttConfigFileInputStream.bufferedReader().use { it.readText() }

        val mqttInfoJsonObject : JsonObject = gson.fromJson(previousMqttConfigFileText, JsonObject::class.java)

        log.info(MQTTConnectionType.FROM_RMS, "setInfoJson : [$setInfoJson]")

        val previousIP : String = mqttInfoJsonObject.get("ip").asString
        val renewalIP : String = setInfoJson.get("mqttIP").asString

        if (previousIP != renewalIP) {
            mqttInfoJsonObject.addProperty("ip", renewalIP)

            val renewFileWriter : FileWriter = FileWriter(previousMqttConfigFile.file)
            renewFileWriter.write(mqttInfoJsonObject.toString())
            renewFileWriter.close()

            log.info(MQTTConnectionType.FROM_RMS, "======= MQTT IP Address {$previousIP} -> {$renewalIP} =======")
            log.warn(MQTTConnectionType.FROM_RMS, "Server Rebooting Required...")
        } else {
            log.warn(MQTTConnectionType.FROM_RMS, "MQTT IP Address is equals to renewalIP {$previousIP} / {$renewalIP}")
        }
    }

    fun handle(envConfig : EnvConfig) {
        val envConfigJson : JsonObject = gson.toJsonTree(envConfig).asJsonObject
        log.info(MQTTConnectionType.FROM_RMS, "envConfigJson : [$envConfigJson]")

        val header : Header? = envConfig.header
        val setInfo : EnvConfigSetInfo? = envConfig.setInfo

        if(header == null) {
            log.error(MQTTConnectionType.FROM_RMS, "envConfig header is null skipping...")
            return
        } else if(setInfo == null) {
            log.error(MQTTConnectionType.FROM_RMS, "envConfig setInfo is null skipping...")
            return
        }

        val headerJson : JsonObject = envConfigJson.getAsJsonObject(KEY_HEADER)
        log.info(MQTTConnectionType.FROM_RMS, "envConfig headerJson : [$headerJson]")

        val setInfoJson : JsonObject = envConfigJson.getAsJsonObject(KEY_SET_INFO)
        log.info(MQTTConnectionType.FROM_RMS, "envConfigsetInfoJson : [$setInfoJson]")

        renewMQTTIP(setInfoJson)

        mqttService.bridge(
            MQTTConnectionType.TO_BRIDGE,
            "/test/config",
            envConfigJson.toString()
        )
    }

    private companion object {
        const val KEY_HEADER : String = "header"
        const val KEY_SET_INFO : String = "setInfo"
    }
}