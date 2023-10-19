package net.wavem.uvc.rms.gateway.path.request

import com.google.gson.Gson
import com.google.gson.JsonArray
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.domain.MQTTConnectionType
import net.wavem.uvc.mqtt.infra.MQTTLogger
import net.wavem.uvc.rms.common.application.UUIDService
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.path.domain.Path
import net.wavem.uvc.rms.gateway.path.domain.PathProperties
import net.wavem.uvc.rms.gateway.path.domain.job.info.PathJobInfo
import net.wavem.uvc.rms.gateway.path.domain.job.path.PathJobPath
import net.wavem.uvc.ros.application.topic.Publisher
import net.wavem.uvc.ros.domain.gps_navigation_msgs.GoalWaypointsStamped
import net.wavem.uvc.ros.domain.builtin_interfaces.Time
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component
import java.text.SimpleDateFormat
import java.sql.Date
import java.time.Instant

@Component
class PathRequestHandler(
    val log : MQTTLogger,
    val pathProperties : PathProperties,
    private val gson : Gson,
    private val uuidService : UUIDService
) {
    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)
    private val rclGoalWaypointsPublisher : Publisher<GoalWaypointsStamped> = Publisher()

    init {
        this.rclGoalWaypointsPublisher.registerPublisher("/gps_slam_navigation/waypoints", GoalWaypointsStamped::class)
        logger.info("RCL {/gps_slam_navigation/waypoints} subscription registered")
    }

    private fun setJobInfoByJson(jobInfoJson : JsonObject) {
        val jobPlanId : String = jobInfoJson.get("jobPlanId").asString
        System.setProperty("jobInfo.jobPlanId", jobPlanId)

        val jobGroupId : String = jobInfoJson.get("jobGroupId").asString
        System.setProperty("jobInfo.jobGroupId", jobGroupId)

        val jobOrderId : String = jobInfoJson.get("jobOrderId").asString
        System.setProperty("jobInfo.jobOrderId", jobOrderId)
    }

    private fun publishPathLocationList(jobPathJson : JsonObject) {
        log.info(MQTTConnectionType.FROM_RMS, "path jobPath $jobPathJson")
        val locationList : JsonArray = jobPathJson.getAsJsonArray("locationList")
        log.info(MQTTConnectionType.FROM_RMS, "path jobPath locationList : $locationList")

        val currentTime : Instant = Instant.now()

        val stamp : Time = Time(currentTime.epochSecond.toInt(), currentTime.nano.toInt())
        val header : Header = Header(stamp, "gps_slam_navigation")

        for (i in 0 .. locationList.size() - 1) {
            log.info(MQTTConnectionType.FROM_RMS, "path jobPath locationList[$i] : ${locationList[i]}")
        }
    }

    fun handle(path : Path) {
        val pathJson : JsonObject = gson.toJsonTree(path).asJsonObject
        log.info(MQTTConnectionType.FROM_RMS, "pathJson : $pathJson")

        val header : Header? = path.header
        val jobInfo : PathJobInfo? = path.jobInfo
        val jobPath : PathJobPath? = path.jobPath

        if(header == null) {
            log.error(MQTTConnectionType.FROM_RMS, "path header is null skipping...")
            return
        } else if (jobInfo == null) {
            log.error(MQTTConnectionType.FROM_RMS, "path jobInfo is null skipping...")
            return
        } else if (jobPath == null) {
            log.error(MQTTConnectionType.FROM_RMS, "path jobPath is null skipping...")
            return
        }

        val headerJson : JsonObject = pathJson.getAsJsonObject(KEY_HEADER)
        log.info(MQTTConnectionType.FROM_RMS, "path headerJson : $headerJson")

        val jobInfoJson : JsonObject = pathJson.getAsJsonObject(KEY_JOB_INFO)
        log.info(MQTTConnectionType.FROM_RMS, "path jobInfoJson : $jobInfoJson")
        this.setJobInfoByJson(jobInfoJson)

        val jobPathJson : JsonObject = pathJson.getAsJsonObject(KEY_JOB_PATH)
        log.info(MQTTConnectionType.FROM_RMS, "path jobPathJson : $jobPathJson")
        this.publishPathLocationList(jobPathJson)
    }

    private companion object {
        const val KEY_HEADER : String = "header"
        const val KEY_JOB_INFO : String = "jobInfo"
        const val KEY_JOB_PATH : String = "jobPath"
        const val KEY_JOB_KIND : String = "jobKind"
    }
}