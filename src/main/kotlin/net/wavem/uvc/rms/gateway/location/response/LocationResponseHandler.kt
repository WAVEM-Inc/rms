package net.wavem.uvc.rms.gateway.location.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.domain.header.Header
import net.wavem.uvc.rms.domain.header.types.RobotType
import net.wavem.uvc.rms.domain.job.JobInfo
import net.wavem.uvc.rms.domain.job.task.TaskInfo
import net.wavem.uvc.rms.domain.job.types.JobGroupType
import net.wavem.uvc.rms.domain.job.types.JobKindType
import net.wavem.uvc.rms.domain.job.types.TaskStatusType
import net.wavem.uvc.rms.gateway.location.domain.Location
import net.wavem.uvc.rms.gateway.location.domain.LocationProperties
import net.wavem.uvc.rms.gateway.location.domain.last_info.LastInfo
import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition
import net.wavem.uvc.rms.gateway.location.domain.types.AreaClsfType
import org.springframework.stereotype.Component

@Component
class LocationResponseHandler(
    private val log: MqttLogger,
    private val locationProperties: LocationProperties,
    private val mqttService: MqttService<String>,
) {

    private fun buildHeader(): Header {
        return Header(
            topicId = "loc0000001",
            robotCorpId = "roc0000001",
            workCorpId = "wco0000001",
            workSiteId = "wst0000001",
            robotId = "rbt0000001",
            robotType = RobotType.AMR.type
        )
    }

    private fun buildJobInfo(): JobInfo {
        val taskInfo: TaskInfo = TaskInfo(
            jobGroup = JobGroupType.SUPPLY.type,
            jobKind = JobKindType.MOVE.type,
            taskStatus = TaskStatusType.ASSIGNED.type
        )

        return JobInfo(
            jobPlanId = "job0000001",
            jobGroupId = 1,
            jobOrderId = 1,
            taskInfo = taskInfo,
            jobResult = null
        )
    }

    private fun buildLastInfo(): LastInfo {
        val locationPosition: LocationPosition = LocationPosition(
            xpos = 11.3245,
            ypos = 24.2214,
            heading = 45
        )

        return LastInfo(
            location = locationPosition,
            areaClsf = AreaClsfType.INDOOR.type,
            floor = "1F",
            batteryLevel = 50,
            velocity = 1.5,
            totalDist = 300
        )
    }

    fun handle() {
        val location: Location = Location(
            header = this.buildHeader(),
            jobInfo = this.buildJobInfo(),
            lastInfo = this.buildLastInfo()
        )

        val locationJSON: JsonObject = Gson().toJsonTree(location).asJsonObject
        log.info(MqttConnectionType.TO_RMS,"location chatterJSON : [${locationJSON} =====")
        mqttService.bridge(MqttConnectionType.FROM_BRIDGE, locationProperties.topic, locationJSON.toString())
    }
}