package net.wavem.uvc.rms.gateway.location.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.common.jwt.JwtService
import net.wavem.uvc.rms.common.types.area.AreaClsfType
import net.wavem.uvc.rms.common.types.header.RobotType
import net.wavem.uvc.rms.common.types.job.JobGroupType
import net.wavem.uvc.rms.common.types.job.JobKindType
import net.wavem.uvc.rms.common.types.job.TaskStatusType
import net.wavem.uvc.rms.gateway.location.domain.Location
import net.wavem.uvc.rms.gateway.location.domain.LocationProperties
import net.wavem.uvc.rms.gateway.location.domain.job.LocationJobInfo
import net.wavem.uvc.rms.gateway.location.domain.last_info.LocationLastInfo
import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition
import net.wavem.uvc.rms.gateway.location.domain.task.LocationTaskInfo
import net.wavem.uvc.ros.RCLKotlin
import net.wavem.uvc.ros.slam.application.SLAMGPSSTransformService
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class LocationResponseHandler(
    private val rclKotlin : RCLKotlin,
    private val locationProperties : LocationProperties,
    private val mqttService : MqttService<String>,
    private val gson : Gson,
    private val jwtService : JwtService,
    private val slamGpsService: SLAMGPSSTransformService
) {
    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    private fun buildHeader() : Header {
        return Header(
            robotCorpId = "roc0000001",
            workCorpId = "wco0000001",
            workSiteId = "wst0000001",
            robotId = "rbt0000001",
            robotType = RobotType.AMR.type
        )
    }

    private fun buildJobInfo() : LocationJobInfo {
        val taskInfo : LocationTaskInfo = LocationTaskInfo(
            jobGroup = JobGroupType.SUPPLY.type,
            jobKind = JobKindType.MOVE.type,
            taskStatus = TaskStatusType.ASSIGNED.type
        )

        return LocationJobInfo(
            jobPlanId = "job0000001",
            jobGroupId = 1,
            jobOrderId = 1,
            taskInfo = taskInfo
        )
    }

    private fun buildLastInfo() : LocationLastInfo {
        val locationPosition : LocationPosition = LocationPosition(
            xpos = 11.3245,
            ypos = 24.2214,
            heading = 45.0
        )

        return LocationLastInfo(
            location = locationPosition,
            areaClsf = AreaClsfType.INDOOR.type,
            floor = "1F",
            batteryLevel = 50,
            velocity = 1.5,
            totalDist = 300
        )
    }

    fun handle() {
        val location : Location = Location(
            header = this.buildHeader(),
            jobInfo = this.buildJobInfo(),
            lastInfo = this.buildLastInfo()
        )

        val locationJSON : JsonObject = Gson().toJsonTree(location).asJsonObject
        logger.info("Location Response JSON : $locationJSON")

        val encryptedJson : String = jwtService.encode("location", locationJSON.toString())

        mqttService.bridge(MqttConnectionType.TO_RMS, locationProperties.topic, locationJSON.toString())
    }
}