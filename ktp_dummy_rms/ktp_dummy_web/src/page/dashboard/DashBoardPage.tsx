import { useEffect, useState } from "react";
import "./DashBoardPage.css";
import MqttService from "../../api/MqttService";
import ResponseComponents from "../../components/response/ResponseComponents";

export default function DashBoardPage() {
    const mqttService: MqttService = new MqttService();

    useEffect(() => {
        mqttService.initialize();

        return () => {
            mqttService.destroy();
        };
    }, []);

    return (
        <div className={"dashboard_container"}>
            <ResponseComponents mqttService={mqttService} />
        </div>
    );
};
