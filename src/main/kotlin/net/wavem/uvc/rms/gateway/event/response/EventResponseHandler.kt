package net.wavem.uvc.rms.gateway.event.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MQTTService
import net.wavem.uvc.mqtt.domain.MQTTConnectionType
import net.wavem.uvc.rms.common.application.TimeService
import net.wavem.uvc.rms.common.application.UUIDService
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.common.domain.job.result.JobResult
import net.wavem.uvc.rms.common.types.area.AreaClsfType
import net.wavem.uvc.rms.common.types.event.ComInfoStatusType
import net.wavem.uvc.rms.common.types.event.EventCodeType
import net.wavem.uvc.rms.common.types.header.RobotType
import net.wavem.uvc.rms.common.types.job.JobGroupType
import net.wavem.uvc.rms.common.types.job.JobKindType
import net.wavem.uvc.rms.common.types.job.JobResultType
import net.wavem.uvc.rms.gateway.event.domain.Event
import net.wavem.uvc.rms.gateway.event.domain.EventProperties
import net.wavem.uvc.rms.gateway.event.domain.com_info.ComInfo
import net.wavem.uvc.rms.gateway.event.domain.event_info.EventInfo
import net.wavem.uvc.rms.gateway.event.domain.job.EventTaskInfo
import net.wavem.uvc.rms.gateway.event.dto.EventInfoDTO
import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition
import net.wavem.uvc.rms.gateway.location.dto.LocationPositionDTO
import net.wavem.uvc.ros.application.topic.Subscription
import net.wavem.uvc.ros.domain.robot_status_msgs.SensorStatus
import net.wavem.uvc.ros.domain.sensor_msgs.BatteryState
import net.wavem.uvc.ros.domain.sensor_msgs.NavSatFix
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class EventResponseHandler(
    private val eventProperties : EventProperties,
    private val mqttService : MQTTService<String>,
    private val timeService : TimeService,
    private val gson : Gson,
    private val uuidService : UUIDService
) {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    private val rclGPSSubscription : Subscription<NavSatFix> = Subscription()
    private val rclBatteryStateSubscription : Subscription<BatteryState> = Subscription()
    private val rclIMUStatusSubscription : Subscription<SensorStatus> = Subscription()
    private val rclScanStatusSubscription : Subscription<SensorStatus> = Subscription()
    private val rclGPSStatusSubscription : Subscription<SensorStatus> = Subscription()
    private val rclBatteryStateStatusSubscription : Subscription<SensorStatus> = Subscription()

    init {
        val rclGpsSubscriptionTopic : String = "/ublox/fix"
        this.rclGPSSubscription.registerSubscription(rclGpsSubscriptionTopic, NavSatFix::class)
        this.logger.info("RCL {$rclGpsSubscriptionTopic} subscription registered")

        val rclBatteryStateSubscriptionTopic : String = "/battery_state"
        this.rclBatteryStateSubscription.registerSubscription(rclBatteryStateSubscriptionTopic, BatteryState::class)
        logger.info("RCL {$rclBatteryStateSubscriptionTopic} subscription registered")

        val rclIMUStatusSubscriptionTopic : String = "/imu/status"
        this.rclIMUStatusSubscription.registerSubscription(rclIMUStatusSubscriptionTopic, SensorStatus::class)
        this.logger.info("RCL {$rclIMUStatusSubscriptionTopic} subscription registered")

        val rclScanStatusSubscriptionTopic : String = "/scan/status"
        this.rclScanStatusSubscription.registerSubscription(rclScanStatusSubscriptionTopic, SensorStatus::class)
        this.logger.info("RCL {$rclScanStatusSubscriptionTopic} subscription registered")

        val rclGPSStatusSubscriptionTopic : String = "/ublox/fix/status"
        this.rclGPSStatusSubscription.registerSubscription(rclGPSStatusSubscriptionTopic, SensorStatus::class)
        this.logger.info("RCL {$rclGPSStatusSubscriptionTopic} subscription registered")

        val rclBatteryStateStatusSubscriptionTopic : String = "/battery_state/status"
        this.rclBatteryStateStatusSubscription.registerSubscription(rclBatteryStateSubscriptionTopic, SensorStatus::class)
        this.logger.info("RCL {$rclBatteryStateStatusSubscriptionTopic} subscription registered")
    }

    private fun buildHeader() : Header {
        return Header(
            robotCorpId = "rco0000001",
            workCorpId = "wco0000001",
            workSiteId = "wst0000001",
            robotId = "rbt0000001",
            robotType = RobotType.AMR.type
        )
    }

    private fun buildJobResult() : JobResult {
        this.rclBatteryStateSubscription.getDataObservable().subscribe {
            val batteryState : BatteryState = BatteryState.read(it)
            logger.info("RCL {/battery_state} subscription callback : $batteryState")
        }

        return JobResult(
            status = JobResultType.SUCCESS.type,
            startTime = timeService.getCurrentDateTime(),
            endTime = timeService.getCurrentDateTime(),
            startBatteryLevel = 70,
            endBatteryLevel = 65,
            dist = 300
        )
    }

    private fun buildJobInfo() : EventTaskInfo {
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

        return EventTaskInfo(
            jobPlanId = jobPlanId,
            jobGroupId = jobGroupId,
            jobOrderId = jobOrderId,
            jobGroup = JobGroupType.SUPPLY.type,
            jobKindType = JobKindType.MOVE.type,
            jobResult = this.buildJobResult()
        )
    }

    private fun buildLocationPosition() : LocationPosition {
        val locationPositionDTO : LocationPositionDTO = LocationPositionDTO()

        this.rclGPSSubscription.getDataObservable().subscribe {
            val navSatFix : NavSatFix = NavSatFix.read(it)
            logger.info("RCL {/ublox/fix} subscription callback : $navSatFix")
            locationPositionDTO.setXpos(navSatFix.latitude)
            locationPositionDTO.setYpos(navSatFix.longitude)
            locationPositionDTO.setHeading(45.0)
        }

        return locationPositionDTO.build()
    }

    private fun buildEventInfo() : EventInfo {
        val eventInfoDTO : EventInfoDTO = EventInfoDTO()
        eventInfoDTO.eventId = uuidService.generateUUID()
        eventInfoDTO.areaClsf = AreaClsfType.INDOOR.type
        eventInfoDTO.floor = "1F"

        this.rclIMUStatusSubscription.getDataObservable().subscribe {
            val imuStatus : SensorStatus = SensorStatus.read(it)

            if (imuStatus.status_code == -1000) {
                eventInfoDTO.eventCd = EventCodeType.BROKEN.type
                eventInfoDTO.eventSubCd = imuStatus.status_message
            }
        }

        this.rclScanStatusSubscription.getDataObservable().subscribe {
            val scanStatus : SensorStatus = SensorStatus.read(it)

            if (scanStatus.status_code == -1001) {
                eventInfoDTO.eventCd = EventCodeType.BROKEN.type
                eventInfoDTO.eventSubCd = scanStatus.status_message
            }
        }

        this.rclGPSStatusSubscription.getDataObservable().subscribe {
            val gpsStatus : SensorStatus = SensorStatus.read(it)

            if (gpsStatus.status_code == -1002) {
                eventInfoDTO.eventCd = EventCodeType.BROKEN.type
                eventInfoDTO.eventSubCd = gpsStatus.status_message
            }
        }

        this.rclBatteryStateStatusSubscription.getDataObservable().subscribe {
            val batteryStatus : SensorStatus = SensorStatus.read(it)

            if (batteryStatus.status_code == -1003) {
                eventInfoDTO.eventCd = EventCodeType.BROKEN.type
                eventInfoDTO.eventSubCd = batteryStatus.status_message
            }
        }

        this.rclBatteryStateSubscription.getDataObservable().subscribe {
            val batteryState : BatteryState = BatteryState.read(it)
            logger.info("RCL {/battery_state} subscription callback : $batteryState")
        }

        eventInfoDTO.location = this.buildLocationPosition()

        return eventInfoDTO.build()
    }

    private fun buildComInfo() : ComInfo {
        return ComInfo(
            status = ComInfoStatusType.CONNECTED.type,
            robotIP = "192.168.1.100",
            mqttIP = mqttService.readMQTTYML().get("ip").asString,
            mqttPort = mqttService.readMQTTYML().get("port").asString
        )
    }

    fun handle() {
        val event : Event = Event(
            header = this.buildHeader(),
            taskInfo = this.buildJobInfo(),
            eventInfo = this.buildEventInfo(),
            comInfo = this.buildComInfo()
        )

        val eventJson : JsonObject = gson.toJsonTree(event).asJsonObject
        mqttService.bridge(MQTTConnectionType.TO_RMS, topic = eventProperties.topic, eventJson.toString())
    }

    companion object {
        const val RCL_GPS_FIX_TOPIC : String = "/ublox/fix"
    }
}