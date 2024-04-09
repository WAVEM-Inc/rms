import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../api/mqttClient";
import * as controlGraphSyncJSON from "../assets/json/control_graphsync.json";
import * as controlMoveToDestJSON from "../assets/json/control_movetodest.json";
import * as controlMsCompleteJSON from "../assets/json/control_mscomplete.json";
import * as cooperativeStartJSON from "../assets/json/cooperative_start.json";
import * as cooperativeStopJSON from "../assets/json/cooperative_stop.json";
import * as errorStatusJSON from "../assets/json/error_status.json";
import * as missionJSON from "../assets/json/mission.json";
import * as obstacleStatusJSON from "../assets/json/obstacle_status.json";
import * as detectedObjectJSON from "../assets/json/detected_object.json";
import RequestComponent from "../components/request/RequestComponent";
import ResponseComponent from "../components/response/ResponseComponent";
import TopComponents from "../components/top/TopComponent";
import { KTP_DEV_ID, getCurrentTime } from "../utils/Utils";
import './DashBoardPage.css';

export default function DashboardPage() {
    const [mqttClient, setMqttClient] = useState<MqttClient | undefined>();
    const [responseData, setResponseData] = useState<any>({});

    const responseTopicFormat: string = "/rms/ktp/dummy/response";

    const requiredResponseTopicList: Array<string> = [
        `${responseTopicFormat}/rbt_status`,
        `${responseTopicFormat}/service_status`,
        `${responseTopicFormat}/error_report`,
        `${responseTopicFormat}/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const requestTopicFormat: string = "/rms/ktp/dummy/request";

    const requiredRequestTopicList: Array<string> = [
        `${requestTopicFormat}/error_status`,
        `${requestTopicFormat}/obstacle/status`,
        `${requestTopicFormat}/obstacle/cooperative`
    ];

    const filterJSON = (json: any): any => {
        const { default: removedDefault, ...newData } = json;

        return newData;
    }

    const onRequestClick = (json: any): void => {
        const newJSON: any = filterJSON(json);

        if (newJSON.data.request_time !== undefined) {
            newJSON.data.request_time = getCurrentTime();
        } else if (newJSON.data.create_time !== undefined) {
            newJSON.data.create_time = getCurrentTime();
        }
        
        const requestJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requestTopicFormat}/common[${newJSON.resource_id}] publish with : ${requestJSONStringified}`);
        mqttClient!.publish(`${requestTopicFormat}/common`, requestJSONStringified);
    }

    const onControlGraphSyncClick = (): void => {
        onRequestClick(controlGraphSyncJSON);
    }

    const onControlMoveToDestClick = (): void => {
        onRequestClick(controlMoveToDestJSON);
    }

    const onControlMsCompleteClick = (): void => {
        onRequestClick(controlMsCompleteJSON);
    }

    const onMissionClick = (): void => {
        onRequestClick(missionJSON);
    }
    
    const onDetectedObjectClick = (): void => {
        onRequestClick(detectedObjectJSON);
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

    const setUpResponseMQTTConnections = (mqttClient: MqttClient): void => {
        console.info(`${requiredResponseTopicList}`);
        for (const requiredTopic of requiredResponseTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic === requiredResponseTopicList[0]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    rbt_status: newData
                }));
            } else if (topic === requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    service_status: newData
                }));
            } else if (topic === requiredResponseTopicList[2]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    error_report: newData
                }));
            } else if (topic === requiredResponseTopicList[3]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    control_report: newData
                }));
            } else if (topic === requiredResponseTopicList[4]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    graph_list: newData
                }));
            } else if (topic === requiredResponseTopicList[5]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    obstacle_detect: newData
                }));
            } else if (topic === requiredResponseTopicList[6]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    lidar_signal: newData
                }));
            } else return;
        });
    }

    useEffect(() => {
        const mqttClient: MqttClient = new MqttClient();
        setMqttClient(mqttClient);

        setUpResponseMQTTConnections(mqttClient);
        handleResponseMQTTCallback(mqttClient);
    }, []);

    return (
        <div className="App">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="response_component_container">
                <ResponseComponent
                responseData={responseData}
                onErrorStatusClick={onErrorStatusClick}
                onObstacleStatusClick={onObstacleStatusClick}
                onCooperativeStartClick={onCooperativeStartClick}
                onCooperativeStopClick={onCooperativeStopClick} />
            </div>
            <div className="request_component_container">
                <RequestComponent 
                onControlGraphSyncClick={onControlGraphSyncClick}
                onControlMoveToDestClick={onControlMoveToDestClick}
                onControlMsCompleteClick={onControlMsCompleteClick}
                onMissionClick={onMissionClick} 
                onDetectedObjectClick={onDetectedObjectClick} />
            </div>
        </div>
    );
};