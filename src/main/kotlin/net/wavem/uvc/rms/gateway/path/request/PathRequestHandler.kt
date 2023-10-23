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
import net.wavem.uvc.ros.domain.builtin_interfaces.Time
import net.wavem.uvc.ros.domain.geometry_msgs.Point
import net.wavem.uvc.ros.domain.geometry_msgs.Pose
import net.wavem.uvc.ros.domain.geometry_msgs.Quaternion
import net.wavem.uvc.ros.domain.gts_navigation_msgs.GoalWaypoints
import net.wavem.uvc.ros.domain.sensor_msgs.NavSatFix
import net.wavem.uvc.ros.domain.sensor_msgs.NavSatStatus
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component
import java.time.Instant

@Component
class PathRequestHandler(
    private val log : MQTTLogger,
    private val gson : Gson
) {
    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)
    private val rclGoalWaypointsPublisher : Publisher<GoalWaypoints> = Publisher()

    init {
        val rclGoalWaypointsTopic : String = "/gts_navigation/waypoints"
        this.rclGoalWaypointsPublisher.registerPublisher(rclGoalWaypointsTopic, GoalWaypoints::class)
        logger.info("RCL {$rclGoalWaypointsTopic} subscription registered")
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

        val stamp : Time = Time(currentTime.epochSecond.toInt(), currentTime.nano)
        val goal_waypoints_list : MutableList<Point> = mutableListOf()

        for ((index, locationJson) in locationList.withIndex()) {
            val location : JsonObject = locationJson.asJsonObject
            log.info(MQTTConnectionType.FROM_RMS, "path jobPath locationList[$index] : $location")

            val xpos : Double = location.get("xpos").asDouble
            val ypos : Double = location.get("ypos").asDouble

            log.info(MQTTConnectionType.FROM_RMS, "path goal_waypoints[$index] xpos : $xpos, ypos : $ypos")

            val goal_waypoints : Point = Point(xpos, ypos, 45.0)
            goal_waypoints_list.add(goal_waypoints)

            log.info(MQTTConnectionType.FROM_RMS, "path goal_waypoints_list[$index] pose ${goal_waypoints_list[index].toString()}")
        }

        log.info(MQTTConnectionType.FROM_RMS, "path goal_waypoints_list : $goal_waypoints_list")

        val goalWaypoints : GoalWaypoints = GoalWaypoints(goal_waypoints_list)
        this.rclGoalWaypointsPublisher.publish(goalWaypoints.write())
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