import { useEffect } from "react";
import MqttService from "../../api/MqttService";
import ResponseComponents from "../../components/response/ResponseComponents";
import "./DashBoardPage.css";
import RequestComponents from "../../components/request/RequestComponents";

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
            <RequestComponents mqttService={mqttService} />
            <ResponseComponents mqttService={mqttService} />
        </div>
    );
};
