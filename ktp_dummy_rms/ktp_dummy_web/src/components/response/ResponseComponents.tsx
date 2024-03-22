import {convertRawJSON, bufferToJSON, getCurrentTime, KTP_DEV_ID} from "../../utils/Utils";
import * as serviceStatusJSON from "../../assets/json/response/service_status.json";
import * as controlReportJSON from "../../assets/json/response/control_report.json"
import * as graphListReportJSON from "../../assets/json/response/graph_list_report.json";
import * as graphListJSON from "../../assets/json/response/graph_list.json";
import * as obstacleDetectJSON from "../../assets/json/response/obstacle_detect.json";
import * as lidarSignalJSON from "../../assets/json/response/lidar_signal.json";
import MqttService from "../../api/MqttService";
import React from "react";

interface ResponseComponentsProps {
    mqttService: MqttService;
}

const ResponseComponents: React.FC<ResponseComponentsProps> = ({ mqttService }: ResponseComponentsProps) => {

    const publishButtonOnClick = async (topic: string, rawJSON: string): Promise<void> => {
        const payloadJSON: any = JSON.parse(rawJSON);

        if (payloadJSON.create_time != null)
        {
            payloadJSON.create_time = getCurrentTime();

            if (topic.includes("service_status"))
            {
                payloadJSON.mission_id = `${KTP_DEV_ID}${payloadJSON.create_time}`;
            }
            else if (topic.includes("control_report") || topic.includes("graph_list_report"))
            {
                payloadJSON.control_id = `c${KTP_DEV_ID}${payloadJSON.create_time}`;
            }
        }

        console.info(`create_time : ${payloadJSON.create_time}`);
        console.info(`ResponseComponent publishButtonOnClick topic : ${topic}, json : ${JSON.stringify(payloadJSON)}`);

        mqttService.publish(topic, JSON.stringify(payloadJSON));
    };

    return (
        <div className={"response_components_container"}>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/service_status", JSON.stringify(serviceStatusJSON))
            }}>
                Service Status
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/control_report", JSON.stringify(controlReportJSON))
            }}>
                Control Report
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/control_report", JSON.stringify(graphListReportJSON))
            }}>
                Graph List Report
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/graph_list", JSON.stringify(graphListJSON))
            }}>
                Graph List
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/obstacle_detect", JSON.stringify(obstacleDetectJSON))
            }}>
                Obstacle Detect
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/lidar_signal", JSON.stringify(lidarSignalJSON))
            }}>
                LiDAR Signal
            </button>
        </div>
    );
}

export default ResponseComponents;