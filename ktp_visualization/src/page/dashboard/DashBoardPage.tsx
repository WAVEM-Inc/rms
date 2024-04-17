import { useEffect, useState } from "react";
import "./DashBoardPage.css";
import TopComponents from "../../components/top/TopComponent";
import MapComponent from "../../components/map/MapComponents";
import RequestComponent from "../../components/request/RequestComponent";
import * as controlGraphSyncJSON from "../../assets/json/control_graphsync.json";
import * as controlMoveToDestJSON from "../../assets/json/control_movetodest.json";
import * as controlMsCompleteJSON from "../../assets/json/control_mscomplete.json";
import * as cooperativeStartJSON from "../../assets/json/cooperative_start.json";
import * as cooperativeStopJSON from "../../assets/json/cooperative_stop.json";
import * as errorStatusJSON from "../../assets/json/error_status.json";
import * as missionJSON from "../../assets/json/mission.json";
import * as obstacleStatusJSON from "../../assets/json/obstacle_status.json";
import MqttClient from "../../api/mqttClient";
import { IPublishPacket } from "mqtt/*";
import { KTP_DEV_ID, getCurrentTime } from "../../utils/Utils";

export default function DashBoardPage() {
    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [pathData, setPathData] = useState<any>(null);
    const [gpsData, setGpsData] = useState<any>(null);

    const requestTopicFormat: string = "/rms/ktp/dummy/request";

    const requiredRequestTopicList: Array<string> = [
        `${requestTopicFormat}/control`,
        `${requestTopicFormat}/mission`,
        `${requestTopicFormat}/detected_object`,
        `${requestTopicFormat}/error_status`,
        `${requestTopicFormat}/obstacle/status`,
        `${requestTopicFormat}/obstacle/cooperative`,
        `${requestTopicFormat}/can/emergency`
    ];

    const filterJSON = (json: any): any => {
        const { default: removedDefault, ...newData } = json;

        return newData;
    }

    const onControlGraphSyncClick = (): void => {
        const newJSON: any = filterJSON(controlGraphSyncJSON);
        newJSON.request_time = getCurrentTime();
        newJSON.control_id = `c${KTP_DEV_ID}${newJSON.request_time}`;

        const controlGraphSyncJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[0]}[GrapySync] publish with : ${controlGraphSyncJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[0], controlGraphSyncJSONStringified);
    }

    const onControlMoveToDestClick = (): void => {
        const newJSON: any = filterJSON(controlMoveToDestJSON);
        newJSON.request_time = getCurrentTime();
        newJSON.control_id = `c${KTP_DEV_ID}${newJSON.request_time}`;

        const controlMoveToDestJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[0]}[MoveToDest] publish with : ${controlMoveToDestJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[0], controlMoveToDestJSONStringified);
    }

    const onControlMsCompleteClick = (): void => {
        const newJSON: any = filterJSON(controlMsCompleteJSON);
        newJSON.request_time = getCurrentTime();
        newJSON.control_id = `c${KTP_DEV_ID}${newJSON.request_time}`;

        const controlMsCompleteJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[0]}[MsComplete] publish with : ${controlMsCompleteJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[0], controlMsCompleteJSONStringified);
    }

    const onMissionClick = (): void => {
        const newJSON: any = filterJSON(missionJSON);
        newJSON.request_time = getCurrentTime();

        const missionJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[1]} publish with : ${missionJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[1], missionJSONStringified);
    }

    const onDetectedObjectClick = (): void => {
        const detecteObjectJSONStringified: string = JSON.stringify(errorStatusJSON);
        console.info(`${requiredRequestTopicList[2]} publish with : ${detecteObjectJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[2], detecteObjectJSONStringified);
    }

    const onErrorStatusClick = (): void => {
        const newJSON: any = filterJSON(errorStatusJSON);

        const errorStatusJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[3]} publish with : ${errorStatusJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[3], errorStatusJSONStringified);
    }

    const onObstacleStatusClick = (): void => {
        const newJSON: any = filterJSON(obstacleStatusJSON);

        const obstacleStatusJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[4]} publish with : ${obstacleStatusJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[4], obstacleStatusJSONStringified);
    }

    const onCooperativeStartClick = (): void => {
        const newJSON: any = filterJSON(cooperativeStartJSON);

        const obstacleCooperativeJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[5]} publish with : ${obstacleCooperativeJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[5], obstacleCooperativeJSONStringified);
    }

    const onCooperativeStopClick = (): void => {
        const newJSON: any = filterJSON(cooperativeStopJSON);

        const obstacleCooperativeJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[5]} publish with : ${obstacleCooperativeJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[5], obstacleCooperativeJSONStringified);
    }

    const onCanEmergencyClick = (): void => {
        const newJSON: any = {
            stop: true
        };

        const canEmergencyJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[6]} publish with : ${canEmergencyJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[6], canEmergencyJSONStringified);
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic === "/rms/ktp/dummy/response/path") {
                console.info(`Path cb : ${JSON.stringify(JSON.parse(payload.toString()))}`);
                setPathData(JSON.parse(payload.toString()));
            } else if (topic === "/rms/ktp/dummy/response/gps") {
                // console.info(`GPS cb : ${JSON.stringify(JSON.parse(payload.toString()))}`);
                setGpsData(JSON.parse(payload.toString()));
            }
        });
    }

    useEffect(() => {
        const mqttClient: MqttClient = new MqttClient();
        setMqttClient(mqttClient);
        mqttClient.subscribe("/rms/ktp/dummy/response/path");
        mqttClient.subscribe("/rms/ktp/dummy/response/gps");
        handleResponseMQTTCallback(mqttClient);
    }, []);

    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="map_component_container">
                <MapComponent pathData={pathData} gpsData={gpsData} />
            </div>
            <div className="request_component_container">
                <RequestComponent
                    onControlGraphSyncClick={onControlGraphSyncClick}
                    onControlMoveToDestClick={onControlMoveToDestClick}
                    onControlMsCompleteClick={onControlMsCompleteClick}
                    onMissionClick={onMissionClick}
                    onDetectedObjectClick={onDetectedObjectClick}
                    onErrorStatusClick={onErrorStatusClick}
                    onObstacleStatusClick={onObstacleStatusClick}
                    onCooperativeStartClick={onCooperativeStartClick}
                    onCooperativeStopClick={onCooperativeStopClick}
                    onCanEmergencyClick={onCanEmergencyClick} />
            </div>
        </div>
    );
}