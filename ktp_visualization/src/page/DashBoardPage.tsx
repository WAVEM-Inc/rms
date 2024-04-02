import { useEffect, useState } from "react";
import TopComponents from "../components/top/TopComponent";
import MqttClient from "../api/mqttClient";
import './DashBoardPage.css';
import { Link } from "react-router-dom";
import RequestComponent from "../components/request/RequestComponent";
import ResponseComponent from "../components/response/ResponseComponent";
import { IPublishPacket } from "mqtt/*";

export default function DashboardPage() {
    const responseTopicFormat: string = "/rms/ktp/dummy/response";

    const requiredTopicList: Array<string> = [
        `${responseTopicFormat}/rbt_status`,
        `${responseTopicFormat}/service_status`,
        `${responseTopicFormat}/error_report`,
        `${responseTopicFormat}/control_report`,
        `${responseTopicFormat}/graph_list`,
        `${responseTopicFormat}/obstacle_detect`,
        `${responseTopicFormat}/lidar_signal`
    ];

    const callbackMap: Map<string, any> = new Map<string, any>();
    const [responseData, setResponseData] = useState<any>({});

    const setUpRequestMQTTConnections = (): void => {
        
    }

    const setUpResponseMQTTConnections = (mqttClient: MqttClient): void => {
        console.info(`${requiredTopicList}`);
        for (const requiredTopic of requiredTopicList) {
            mqttClient.subscribe(requiredTopic);
        }
    }

    const handleResponseMQTTCallback = (mqttClient: MqttClient): void => {
        mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
            if (topic == requiredTopicList[0]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    rbt_status: newData
                }));
            } else if (topic == requiredTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    service_status: newData
                }));
            } else if (topic == requiredTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    error_report: newData
                }));
            } else if (topic == requiredTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    control_report: newData
                }));
            } else if (topic == requiredTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    graph_list: newData
                }));
            } else if (topic == requiredTopicList[1]) {
                const newData: any = JSON.parse(payload.toString());
                setResponseData((prevData: any) => ({
                    ...prevData,
                    obstacle_detect: newData
                }));
            } else if (topic == requiredTopicList[1]) {
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

        setUpResponseMQTTConnections(mqttClient);
        handleResponseMQTTCallback(mqttClient);
    }, []);

    return (
        <div className="App">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="request_component_container">
                <RequestComponent />
            </div>
            <div className="response_component_container">
                <ResponseComponent responseData={responseData} />
            </div>
        </div>
    );
};