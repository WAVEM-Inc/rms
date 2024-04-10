import { Map, MapMarker } from "react-kakao-maps-sdk";
import "./MapComponents.css";

const MapComponent = () => {
    return (
        <div className="map_components">
            <Map
                className="map_container"
                center={{ lat: 37.306595, lng: 127.240111 }}>
            </Map>
        </div>
    );
};

export default MapComponent;