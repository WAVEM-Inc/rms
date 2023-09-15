package net.wavem.uvc.rms.gateway.path.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.path.domain.Path
import net.wavem.uvc.rms.gateway.path.domain.PathProperties
import net.wavem.uvc.rms.gateway.path.domain.job.info.PathJobInfo
import net.wavem.uvc.rms.gateway.path.domain.job.kind.PathJobKind
import net.wavem.uvc.rms.gateway.path.domain.job.path.PathJobPath
import org.springframework.stereotype.Component

@Component
class PathRequestHandler(
    val log: MqttLogger,
    val pathProperties: PathProperties,
    val mqttService: MqttService<String>
) {

    fun handle(path: Path) {
        val pathJson: JsonObject = Gson().toJsonTree(path).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "pathJson : [$pathJson]")

        val header: Header? = path.header
        val jobInfo: PathJobInfo? = path.jobInfo
        val jobPath: PathJobPath? = path.jobPath
        val jobKind: PathJobKind? = path.jobKind

        if(header == null) {
            log.error(MqttConnectionType.FROM_RMS, "path header is null skipping...")
            return
        } else if (jobInfo == null) {
            log.error(MqttConnectionType.FROM_RMS, "path jobInfo is null skipping...")
            return
        } else if (jobPath == null) {
            log.error(MqttConnectionType.FROM_RMS, "path jobPath is null skipping...")
            return
        } else if (jobKind == null) {
            log.error(MqttConnectionType.FROM_RMS, "path jobKind is null skipping...")
            return
        }

        val headerJson: JsonObject = pathJson.getAsJsonObject(KEY_HEADER)
        log.info(MqttConnectionType.FROM_RMS, "path headerJson : [$headerJson]")

        val jobInfoJson: JsonObject = pathJson.getAsJsonObject(KEY_JOB_INFO)
        log.info(MqttConnectionType.FROM_RMS, "path jobInfoJson : [$jobInfoJson]")

        val jobPathJson: JsonObject = pathJson.getAsJsonObject(KEY_JOB_PATH)
        log.info(MqttConnectionType.FROM_RMS, "path jobPathJson : [$jobPathJson]")

        val jobKindJson: JsonObject = pathJson.getAsJsonObject(KEY_JOB_KIND)
        log.info(MqttConnectionType.FROM_RMS, "path jobKindJson : [$jobKindJson]")

        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            topic = "/test/path",
            pathJson.toString()
        )
    }

    private companion object {
        const val KEY_HEADER: String = "header"
        const val KEY_JOB_INFO: String = "jobInfo"
        const val KEY_JOB_PATH: String = "jobPath"
        const val KEY_JOB_KIND: String = "jobKind"
    }
}