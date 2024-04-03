import mqtt, { IClientOptions } from 'mqtt';
import * as mqttJSON from "../assets/config/mqtt.json";

export default class MqttClient {

    broker_url: string;
    connect_opts: IClientOptions;
    client: mqtt.MqttClient;

    constructor() {
        const mqtt_json: any = JSON.parse(JSON.stringify(mqttJSON));
        console.info(`MQTT : ${JSON.stringify(mqtt_json)}`);

        this.connect_opts = {
            host: mqtt_json.host,
            port: mqtt_json.port,
            protocol: mqtt_json.protocol,
            username: mqtt_json.username,
            password: mqtt_json.password
        };

        this.broker_url = `${mqtt_json.protocol}://${mqtt_json.host}:${mqtt_json.port}/mqtt`;
        this.client = mqtt.connect(this.broker_url, this.connect_opts);
        this.onConnect();
    };

    private onConnect(): void {
        this.client!.on("connect", () => {
            if (this.client!.connected) {
                console.log(`[MQTT] connected with [${this.broker_url}]`);
            }
            else {
                console.error('[MQTT] connection disconnected');
            }
        });
        this.client!.on("error", (err) => {
            console.error(`[MQTT] connection on ${err}`);
        });
    };

    public publish(topic: string, message: string): void {
        try {
            this.client!.publish(topic, message);
        } catch (error: any) {
            console.error(`[MQTT] publishing errror : ${error}`);
            throw new Error(error);
        };
    };

    public subscribe(topic: string): void {
        try {
            this.client!.subscribe(topic, function (err: Error | null, granted: mqtt.ISubscriptionGrant[] | undefined) {
                if (err) {
                    console.error(`[MQTT] ${topic} subscription on ${err}`);
                    return;
                };
                console.log(`[MQTT] subscription has granted by topic {${granted![0].topic}}`);
            });
        } catch (error: any) {
            console.error(`[MQTT] {${topic}} subscription : ${error}`);
            throw new Error(error);
        };
    };
};