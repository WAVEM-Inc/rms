import { useEffect } from "react";
import "./DashBoardPage.css";
import TopComponents from "../../components/top/TopComponent";
import MapComponent from "../../components/map/MapComponents";

export default function DashBoardPage() {
    return (
        <div className="dash_board_container">
            <div className="top_component_container">
                <TopComponents />
            </div>
            <div className="map_component_container">
                <MapComponent />
            </div>
        </div>
    );
}