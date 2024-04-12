import { useEffect, useRef, useState } from "react";
import "./MapComponents.css";

interface MapComponentProps {
    pathData: any;
    gpsData: any;
}

const MapComponent = ({ pathData, gpsData }: MapComponentProps) => {
    const { naver } = window;
    const mapRef: React.MutableRefObject<null> = useRef(null);
    let map: naver.maps.Map;

    let pathMarkerArray: Array<naver.maps.Marker> = [];

    const addPathMarker = (node: any): any => {
        console.info(`position lat : ${node.position.latitude}, lon : ${node.position.longitude}`);

        const marker: naver.maps.Marker = new naver.maps.Marker({
            position: new naver.maps.LatLng(node.position.latitude, node.position.longitude),
            // position: new naver.maps.LatLng(37.3060542, 127.2399165),
            map: map,
            title: node.node_id
        });

        return marker;
    }

    useEffect(() => {
        if (mapRef.current && naver) {
            map = new naver.maps.Map(mapRef.current, {
                // center: new naver.maps.LatLng(37.3060542, 127.2399165),
                center: new naver.maps.LatLng(36.1137155, 128.3676005),
                mapTypeId: naver.maps.MapTypeId.HYBRID,
                zoom: 19
            });

            // if (mapRef.current && naver && gpsData) {
            //     const marker: naver.maps.Marker = new naver.maps.Marker({
            //         position: new naver.maps.LatLng(gpsData.latitude, gpsData.longitude),
            //         // position: new naver.maps.LatLng(37.3060542, 127.2399165),
            //         map: map,
            //         title: "Current"
            //     });
            // }

            if (mapRef.current && naver && pathData) {
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
    }, [naver, pathData]);

    useEffect(() => {
        console.info(`gps : ${JSON.stringify(gpsData)}`);
    }, [gpsData]);


    return (
        <div className="map_components">
            <div className="map_container">
                <div ref={mapRef} id="map" />
            </div>
        </div>
    );
};

export default MapComponent;