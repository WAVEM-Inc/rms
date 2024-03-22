import mqtt, { OnMessageCallback } from "mqtt";
import {convertRawJSON} from "../utils/Utils";
import * as mqttConfigJSON from "../assets/params/mqttConfig.json";

export default class MqttService
{
    private readonly brokerURL: string;
    private client?: mqtt.MqttClient;

    constructor()
    {
        const configJSON: any = convertRawJSON(mqttConfigJSON);
        console.info(`configJSON : ${JSON.stringify(configJSON)}`);
        this.brokerURL = `${configJSON.protocol}://${configJSON.host}:${configJSON.port}`;
        console.info(`brokerURL : ${this.brokerURL}`);
        this.client = undefined;
    }
    
    public initialize(): void
    {
        this.client = mqtt.connect(this.brokerURL);
        this.client!.options.clientId = "12361251235";
        this.onConnect();
    }

    private onConnect(): void
    {
        this.client!.on("connect", () => {
            if(!this.client!.connected)
            {
                console.error("[MQTT] connection discarded");
            }
            else
            {
                console.info(`[MQTT] connected with {${this.brokerURL}}`);
            }
        });

        this.client!.on("error", (err) => {
            console.error(`[MQTT] connection on ${err}`);
            this.client!.end();
        });
    }

    public destroy(): void
    {
        this.client!.end();
    }

    public publish(topic : string, message : string): void
    {
        try
        {
            this.client!.publish(topic, message);
        }
        catch (error : any) {
            console.error(`[MQTT] publishing errror : ${error}`);
            return;
        }
    }

    public subscribe(topic : string, callback: OnMessageCallback): void
    {
        try
        {
            this.client!.subscribe(topic, (err: any, granted: any) => {
                if (err) {
                    console.error(`[MQTT] ${topic} subscription on ${err}`);
                    return;
                };
                console.info(`[MQTT] subscription has granted by topic {${granted[0].topic}}`);
            });

            this.client!.on("message", callback);
        }
        catch (error : any) {
            console.error(`[MQTT] {${topic}} subscription : ${error}`);
            return;
        }
    }
};