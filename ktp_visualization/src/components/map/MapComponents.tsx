import { useEffect, useRef, useState } from "react";
import "./MapComponents.css";
import {
    RenderAfterNavermapsLoaded,
    NaverMap,
    Marker,
    Polyline,
    Polygon,
} from 'react-naver-maps';

const MapComponent = () => {
    const { naver } = window;
    const mapRef: React.MutableRefObject<null> = useRef(null);

    useEffect(() => {
        if (mapRef.current && naver) {
            const map: naver.maps.Map = new naver.maps.Map(mapRef.current, {
                center: new naver.maps.LatLng(37.3060542, 127.2399165),
                mapTypeId: naver.maps.MapTypeId.HYBRID,
                zoom: 20
            });

            const marker1: naver.maps.Marker = new naver.maps.Marker({
                position: new naver.maps.LatLng(37.3060542, 127.2399165),
                map: map,
                title: "TEST1"
            });

            const marker2: naver.maps.Marker = new naver.maps.Marker({
                position: new naver.maps.LatLng(37.3061042, 127.2399165),
                map: map,
                title: "TEST2"
            });

            naver.maps.Event.addListener(marker1, "click", () => {
                console.info(`marker : ${marker1.getTitle()}`);
                console.info(`marker 1 : ${marker1.getPosition().x}`);
            });

            const polyline: naver.maps.Polyline = new naver.maps.Polyline({
                map: map,
                path: [
                    marker1.getPosition(),
                    marker2.getPosition()
                ]
            });
        }
    }, []);

    return (
        <div className="map_components">
            <div ref={mapRef} id="map" style={{ width: "1400px", height: "800px" }} />
        </div>
    );
};

export default MapComponent;