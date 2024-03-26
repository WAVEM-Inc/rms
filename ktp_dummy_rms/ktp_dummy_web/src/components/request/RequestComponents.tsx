import React from "react";
import MqttService from "../../api/MqttService";
import * as controlGrapySyncJSON from "../../assets/json/request/control_graphsync.json";
import * as controlMoveToDestJSON from "../../assets/json/request/control_movetodest.json";
import * as controlMsCompleteJSON from "../../assets/json/request/control_mscomplete.json";
import * as detectedObjectJSON from "../../assets/json/request/detected_object.json";
import * as missionJSON from "../../assets/json/request/mission.json";
import { KTP_DEV_ID, getCurrentTime } from "../../utils/Utils";

interface RequestComponentProps {
    mqttService: MqttService;
}

const RequestComponents: React.FC<RequestComponentProps> = ({ mqttService }: RequestComponentProps) => {

    const publishButtonOnClick = async (topic: string, rawJSON: string): Promise<void> => {
        const payloadJSON: any = JSON.parse(rawJSON);

        if (payloadJSON.request_time != null)
        {
            payloadJSON.request_time = getCurrentTime();
            console.info(`request_time : ${payloadJSON.request_time}`);

            if (topic.includes("control"))
            {
                payloadJSON.mission_id = `c${KTP_DEV_ID}${payloadJSON.request_time}`;

                let parsedJSON: any = {};
                parsedJSON.control = payloadJSON;

                console.info(`RequestComponents publishButtonOnClick topic : ${topic}, json : ${JSON.stringify(parsedJSON)}`);
                mqttService.publish(topic, JSON.stringify(parsedJSON));
            }
            else if (topic.includes("mission"))
            {
                payloadJSON.mission_id = `${KTP_DEV_ID}${payloadJSON.request_time}`;
                payloadJSON.task[0].task_id = payloadJSON.mission_id;

                let parsedJSON: any = {};
                parsedJSON.mission = payloadJSON;

                console.info(`RequestComponents publishButtonOnClick topic : ${topic}, json : ${JSON.stringify(parsedJSON)}`);
                mqttService.publish(topic, JSON.stringify(parsedJSON));
            }
        }
        else if (payloadJSON.create_time != null)
        {
            payloadJSON.create_time = getCurrentTime();
            console.info(`create_time : ${payloadJSON.create_time}`);
            console.info(`RequestComponents publishButtonOnClick topic : ${topic}, json : ${JSON.stringify(payloadJSON)}`);
            mqttService.publish(topic, JSON.stringify(payloadJSON));
        }
    }

    return (
        <div className={"request_components_container"}>
            <button onClick={() => {
                publishButtonOnClick("/ktp_data_manager/assign/control", JSON.stringify(controlMoveToDestJSON));
            }}>
                Control - MoveToDest
            </button>
            <button onClick={() => {
                publishButtonOnClick("/ktp_data_manager/assign/control", JSON.stringify(controlMsCompleteJSON));
            }}>
                Control - MsComplete
            </button>
            <button onClick={() => {
                publishButtonOnClick("/ktp_data_manager/assign/control", JSON.stringify(controlGrapySyncJSON));
            }}>
                Control - GraphSync
            </button>
            <button onClick={() => {
                publishButtonOnClick("/ktp_data_manager/assign/mission", JSON.stringify(missionJSON));
            }}>
                Mission
            </button>
            <button onClick={() => {
                publishButtonOnClick("/rms/ktp/data/detected_object", JSON.stringify(detectedObjectJSON));
            }}>
                Detected Object
            </button>
        </div>
    );
}

export default RequestComponents;