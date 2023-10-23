package net.wavem.uvc.rms.gateway.location.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MQTTService
import net.wavem.uvc.mqtt.domain.MQTTConnectionType
import net.wavem.uvc.rms.common.domain.header.Header
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
import net.wavem.uvc.rms.gateway.location.dto.LocationPositionDTO
import net.wavem.uvc.ros.application.topic.Subscription
import net.wavem.uvc.ros.domain.sensor_msgs.NavSatFix
import net.wavem.uvc.ros.domain.sensor_msgs.BatteryState
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class LocationResponseHandler(
    private val locationProperties : LocationProperties,
    private val mqttService : MQTTService<String>,
    private val gson : Gson
) {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)
    private val rclGPSSubscription : Subscription<NavSatFix> = Subscription()
    private val rclBatteryStateSubscription : Subscription<BatteryState> = Subscription()

    init {
        val rclGPSSubscriptionTopic : String = "/ublox/fix"
        this.rclGPSSubscription.registerSubscription(rclGPSSubscriptionTopic, NavSatFix::class)
        logger.info("RCL {$rclGPSSubscriptionTopic} subscription registered")

        val rclBatteryStateSubscriptionTopic : String = "/battery_state"
        this.rclBatteryStateSubscription.registerSubscription(rclBatteryStateSubscriptionTopic, BatteryState::class)
        logger.info("RCL {$rclBatteryStateSubscriptionTopic} subscription registered")
    }

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

        var jobPlanId : String = ""
        val jobPlanIdFromYML : String? = System.getProperty("jobInfo.jobPlanId")
        if (jobPlanIdFromYML != null) {
            jobPlanId = jobPlanIdFromYML
        }

        var jobGroupId : String = ""
        val jobGroupIdFromYML : String? = System.getProperty("jobInfo.jobGroupId")
        if (jobGroupIdFromYML != null) {
            jobGroupId = jobGroupIdFromYML
        }

        var jobOrderId : String = ""
        val jobOrderIdFromYML : String? = System.getProperty("jobInfo.jobOrderId")
        if (jobOrderIdFromYML != null) {
            jobOrderId = jobOrderIdFromYML
        }

        return LocationJobInfo(
            jobPlanId = jobPlanId,
            jobGroupId = jobGroupId,
            jobOrderId = jobOrderId,
            taskInfo = taskInfo
        )
    }

    private fun buildLastInfo() : LocationLastInfo {
        val locationPositionDTO : LocationPositionDTO = LocationPositionDTO()

        this.rclGPSSubscription.getDataObservable().subscribe {
            val navSatFix : NavSatFix = NavSatFix.read(it)
            locationPositionDTO.xpos = navSatFix.latitude
            locationPositionDTO.ypos = navSatFix.longitude
            locationPositionDTO.heading = 45.0
        }

        this.rclBatteryStateSubscription.getDataObservable().subscribe {
            val batteryState : BatteryState = BatteryState.read(it)
        }

        val locationPosition : LocationPosition = locationPositionDTO.build()

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

        val locationJSON : JsonObject = gson.toJsonTree(location).asJsonObject

        mqttService.bridge(MQTTConnectionType.TO_RMS, locationProperties.topic, locationJSON.toString())
    }
}