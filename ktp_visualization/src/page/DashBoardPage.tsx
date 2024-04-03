import { IPublishPacket } from "mqtt/*";
import { useEffect, useState } from "react";
import MqttClient from "../api/mqttClient";
import * as controlGraphSyncJSON from "../assets/json/control_graphsync.json";
import * as controlMoveToDestJSON from "../assets/json/control_movetodest.json";
import * as controlMsCompleteJSON from "../assets/json/control_movetodest.json";
import * as detectedObjectJSON from "../assets/json/detected_object.json";
import * as missionJSON from "../assets/json/mission.json";
import RequestComponent from "../components/request/RequestComponent";
import ResponseComponent from "../components/response/ResponseComponent";
import TopComponents from "../components/top/TopComponent";
import './DashBoardPage.css';
import { getCurrentTime } from "../utils/Utils";

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
        `${requestTopicFormat}/control`,
        `${requestTopicFormat}/mission`,
        `${requestTopicFormat}/detected_object`
    ];

    const filterJSON = (json: any): any => {
        const { default: removedDefault, ...newData } = json;

        return newData;
    }

    const onControlGraphSyncClick = (): void => {
        const newJSON: any = filterJSON(controlGraphSyncJSON);
        newJSON.request_time = getCurrentTime();

        const controlGraphSyncJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[0]}[GrapySync] publish with : ${controlGraphSyncJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[0], controlGraphSyncJSONStringified);
    }

    const onControlMoveToDestClick = (): void => {
        const newJSON: any = filterJSON(controlMoveToDestJSON);
        newJSON.request_time = getCurrentTime();

        const controlMoveToDestJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[0]}[MoveToDest] publish with : ${controlMoveToDestJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[0], controlMoveToDestJSONStringified);
    }

    const onControlMsCompleteClick = (): void => {
        const newJSON: any = filterJSON(controlMsCompleteJSON);
        newJSON.request_time = getCurrentTime();

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
        const newJSON: any = filterJSON(detectedObjectJSON);
        newJSON.request_time = getCurrentTime();

        const detecteObjectJSONStringified: string = JSON.stringify(newJSON);
        console.info(`${requiredRequestTopicList[2]} publish with : ${detecteObjectJSONStringified}`);
        mqttClient!.publish(requiredRequestTopicList[2], detecteObjectJSONStringified);
    }

    const setUpResponseMQTTConnections = (mqttClient: MqttClient): void => {
        console.info(`${requiredResponseTopicList}`);
        for (const requiredTopic of requiredResponseTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic == requiredResponseTopicList[0]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    rbt_status: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    service_status: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    error_report: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    control_report: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    graph_list: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    obstacle_detect: newData
                }));
            } else if (topic == requiredResponseTopicList[1]) {
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
                <ResponseComponent responseData={responseData} />
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