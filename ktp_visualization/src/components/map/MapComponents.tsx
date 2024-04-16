import { useEffect, useRef, useState } from "react";
import "./MapComponents.css";

interface MapComponentProps {
    pathData: any;
    gpsData: any;
}

const MapComponent = ({ pathData, gpsData }: MapComponentProps) => {
    const { naver } = window;
    const mapRef: React.MutableRefObject<null> = useRef(null);

    let map: naver.maps.Map | null = null;
    const [currMarker, setCurrMarker] = useState<naver.maps.Marker | null>(null);
    const [currentGps, setCurrentGps] = useState<any>({
        latitude: 0.0,
        longitude: 0.0
    });

    let pathMarkerArray: Array<naver.maps.Marker> = [];

    const [currentMode, setCurrentMode] = useState<string>("KEC");

    const wkValveCoord: naver.maps.LatLng = new naver.maps.LatLng(35.157851, 128.858111);
    const blueSpaceCoord: naver.maps.LatLng = new naver.maps.LatLng(37.3060542, 127.2399165);
    const kecCoord: naver.maps.LatLng = new naver.maps.LatLng(36.1137155, 128.3676005);

    const initializeMap = (): void => {
        const openStreetMapType: naver.maps.ImageMapType = new naver.maps.ImageMapType({
            name: "OSM",
            minZoom: 0,
            maxZoom: 19,
            tileSize: new naver.maps.Size(256, 256),
            projection: naver.maps.EPSG3857,
            repeatX: true,
            tileSet: [
                "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
                "https://b.tile.openstreetmap.org/{z}/{x}/{y}.png",
                "https://c.tile.openstreetmap.org/{z}/{x}/{y}.png"
            ],
            provider: [{
                title: " /OpenStreetMap",
                link: "http://www.openstreetmap.org/copyright"
            }]
        });

        const mapOpts: any = {
            center: kecCoord,
            mapTypeId: naver.maps.MapTypeId.HYBRID,
            zoom: 19,
            zoomControl: true,
            zoomControlOptions: {
                style: naver.maps.ZoomControlStyle.SMALL,
                position: naver.maps.Position.TOP_RIGHT,
            },
            mapTypeControl: true,
            mapTypeControlOptions: {
                style: naver.maps.MapTypeControlStyle.BUTTON
            }
        }
        map = new naver.maps.Map(mapRef.current, mapOpts);
        map!.mapTypes.set("osm", openStreetMapType);

        naver.maps.Event.once(map, 'init', function () {
            const controlHtml: string =
            `
            <div style="position: absolute; z-index: 100; margin: 0px; padding: 0px; pointer-events: none; top: 0px;">
                <div style="border: 0px; margin: 0px; padding: 0px; pointer-events: none; float: left;"">
                    <div style="position: relative; z-index: 69; margin: 10px; pointer-events: auto;">
                        <ul class="move_btn_ul">
                            <li class="move_btn_li"><button class="move_btn">부산 원광밸브</button></li>
                            <li class="move_btn_li"><button class="move_btn">용인 블루스페이스</button></li>
                            <li class="move_btn_li"><button class="move_btn">KEC 구미</button></li>
                        </ul>
                    </div>
                </div>
            </div>
            `;
            const moveControls: naver.maps.CustomControl = new naver.maps.CustomControl(controlHtml, {
                position: naver.maps.Position.TOP_LEFT
            });
            moveControls.setMap(map);

            naver.maps.Event.addDOMListener(moveControls.getElement(), 'click', function (e: Event) {
                const target: HTMLElement = e.target as HTMLElement;
                const locationName: string = target.textContent || "";

                switch (locationName.trim()) {
                    case "부산 원광밸브":
                        setCurrentMode("WkValve");
                        map!.setCenter(wkValveCoord);
                        break;
                    case "용인 블루스페이스":
                        setCurrentMode("BlueSpace");
                        map!.setCenter(blueSpaceCoord);
                        break;
                    case "KEC 구미":
                        setCurrentMode("KEC");
                        map!.setCenter(kecCoord);
                        break;
                    default:
                        break;
                }
            });
        });
    }

    const addPathMarker = (node: any): any => {
        console.info(`position lat : ${node.position.latitude}, lon : ${node.position.longitude}`);

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            map: map,
            title: node.node_id
        });

        return marker;
    }

    useEffect(() => {
        if (mapRef.current && naver && !map) {
            initializeMap();
        } else return;

        const initialCurrMarker: naver.maps.Marker = new naver.maps.Marker({
            position: map!.getCenter(),
            map: map,
            title: "RobotCurrentPos",
            icon: {
                url: process.env.PUBLIC_URL + "location-dot-solid.png",
                size: new naver.maps.Size(25, 34),
                scaledSize: new naver.maps.Size(25, 34),
                origin: new naver.maps.Point(0, 0),
                anchor: new naver.maps.Point(12, 34)
            }
        });
        setCurrMarker(initialCurrMarker);
    }, [naver, map]);

    useEffect(() => {
        if (gpsData) {
            setCurrentGps({
                latitude: gpsData.latitude,
                longitude: gpsData.longitude
            });

            currMarker!.setPosition(new naver.maps.LatLng(gpsData.latitude, gpsData.longitude));
            currMarker!.setAnimation(naver.maps.Animation.BOUNCE);
        }
    }, [gpsData]);

    useEffect(() => {
        if (mapRef.current && naver) {
            if (pathData) {
                const nodeList: Array<any> = pathData.node_list;

                for (const node of nodeList) {
                    console.info(`Node : ${JSON.stringify(node)}`);
                    const marker: naver.maps.Marker = addPathMarker(node);
                    pathMarkerArray.push(marker);
                }

                let path: Array<any> = [];
                for (const pathMarker of pathMarkerArray) {
                    path.push(pathMarker.getPosition());
                }

                const polyline: naver.maps.Polyline = new naver.maps.Polyline({
                    map: map,
                    path: path
                });
            }
        }
    }, [pathData]);

    useEffect(() => {
        return () => {
            if (map) {
                setCurrMarker(null);
                map.destroy();
                map = null;
            }
        }
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div ref={mapRef} id={"map"} />
                <div className={"gps_data_container"}>
                    longitude : {currentGps.longitude}
                    <br></br>
                    latitude : {currentGps.latitude}
                </div>
            </div>
        </div>
    );
};

export default MapComponent;