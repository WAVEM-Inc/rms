package net.wavem.uvc.rms.gateway.event.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.common.application.TimeService
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.common.domain.job.result.JobResult
import net.wavem.uvc.rms.common.jwt.JwtService
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
import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition
import net.wavem.uvc.ros.RCLKotlin
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component
import sensor_msgs.msg.dds.NavSatFix
import sensor_msgs.msg.dds.NavSatFixPubSubType
import us.ihmc.ros2.ROS2Subscription

@Component
class EventResponseHandler(
    private val rclKotlin : RCLKotlin,
    private val eventProperties : EventProperties,
    private val mqttService : MqttService<String>,
    private val timeService : TimeService,
    private val gson : Gson,
    private val jwtService : JwtService
) {

    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)
    private var rclGpsSubscription : ROS2Subscription<NavSatFix>? = null
    private var rclNavSatFix : NavSatFix? = null

    private fun setRCLNavSatFix(navSatFix : NavSatFix) {
        this.rclNavSatFix = navSatFix
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
        return EventTaskInfo(
            jobPlanId = "job0000001",
            jobGroupId = 1,
            jobOrderId = 1,
            jobGroup = JobGroupType.SUPPLY.type,
            jobKindType = JobKindType.MOVE.type,
            jobResult = this.buildJobResult()
        )
    }

    private fun buildLocationPosition() : LocationPosition {
        if (rclKotlin.isRCLKotlinInitialized()) {
            if (rclGpsSubscription == null) {
                rclGpsSubscription = rclKotlin.getRCLNode().createSubscription(NavSatFixPubSubType(), { it ->
                    val navSatFixCallback : NavSatFix = it.readNextData()
                    setRCLNavSatFix(navSatFixCallback)
                }, RCL_GPS_FIX_TOPIC)

                logger.info("{$RCL_GPS_FIX_TOPIC} subscription registered")
            }
        } else {
            logger.error("Failed to initialize RCLKotlin")
        }

        if (this.rclNavSatFix == null) {
            logger.error("{$RCL_GPS_FIX_TOPIC} callback is null")
        } else {
            logger.info("{$RCL_GPS_FIX_TOPIC} callback returned : ${rclNavSatFix.toString()}")
        }

        return LocationPosition(
            xpos = 11.3245,
            ypos = 24.2214,
            heading = 45.0
        )
    }

    private fun buildEventInfo() : EventInfo {
        return EventInfo(
            eventId = "evt0000001",
            eventCd = EventCodeType.STOP.type,
            eventSubCd = "SUB001",
            areaClsf = AreaClsfType.INDOOR.type,
            floor = "1F",
            batteryLevel = 50,
            location = this.buildLocationPosition()
        )
    }

    private fun buildComInfo() : ComInfo {
        return ComInfo(
            status = ComInfoStatusType.CONNECTED.type,
            robotIP = "192.168.1.100",
            mqttIP = "192.168.1.200",
            mqttPort = mqttService.readMQTTYML().get("port").asString
//            mqttIP = mqttService.readMQTTYML().get("ip").asString,
//            mqttPort = mqttService.readMQTTYML().get("port").asString
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
        logger.info("Event Response JSON : $eventJson")
        val encryptedJson : String = jwtService.encode("event", eventJson.toString())

        mqttService.bridge(MqttConnectionType.TO_RMS, topic = eventProperties.topic, eventJson.toString())
    }

    companion object {
        const val RCL_GPS_FIX_TOPIC : String = "/ublox/fix"
    }
}