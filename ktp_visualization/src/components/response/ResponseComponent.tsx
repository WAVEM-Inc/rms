import React, { useEffect, useState } from "react";
import './ResponseComponent.css';

interface ResponseComponentProps {
    responseData: any;
}

const ResponseComponent: React.FC<ResponseComponentProps> = ({ responseData }: ResponseComponentProps) => {
    const [rbtStatusExpanded, setRbtStatusExpanded] = useState<boolean>(false);
    const [serviceStatusExpanded, setServiceStatusExpanded] = useState<boolean>(false);
    const [errorReportExpanded, setErrorReportExpanded] = useState<boolean>(false);
    const [controlReportExpanded, setControlReportExpanded] = useState<boolean>(false);
    const [graphListExpanded, setGraphListExpanded] = useState<boolean>(false);
    const [obstacleDetectExpanded, setObstacleDetectExpanded] = useState<boolean>(false);
    const [lidarSignalExpanded, setLidarSignalExpanded] = useState<boolean>(false);

    const [rbtStatus, setRbtStatus] = useState<string>("");
    const [serviceStatus, setServiceStatus] = useState<string>("");
    const [errorReport, setErrorReport] = useState<string>("");
    const [controlReport, setControlReport] = useState<string>("");
    const [graphList, setGraphList] = useState<string>("");
    const [obstacleDetect, setObstacleDetect] = useState<string>("");
    const [lidarSignal, setLidarSignal] = useState<string>("");

    const toggleRbtStatus = () => {
        setRbtStatusExpanded(!rbtStatusExpanded);
    }

    const toggleServiceStatus = () => {
        setServiceStatusExpanded(!serviceStatusExpanded);
    }

    const toggleErrorReport = () => {
        setErrorReportExpanded(!errorReportExpanded);
    }

    const toggleControlReport = () => {
        setControlReportExpanded(!controlReportExpanded);
    }

    const toggleGraphList = () => {
        setGraphListExpanded(!graphListExpanded);
    }

    const toggleObstacleDetect = () => {
        setObstacleDetectExpanded(!obstacleDetectExpanded);
    }

    const toggleLidarSignal = () => {
        setLidarSignalExpanded(!lidarSignalExpanded);
    }

    useEffect(() => {
        setRbtStatus(JSON.stringify(responseData.rbt_status, null, 4));
        setServiceStatus(JSON.stringify(responseData.service_status, null, 4));
        setErrorReport(JSON.stringify(responseData.error_report, null, 4));
        setControlReport(JSON.stringify(responseData.control_report, null, 4));
        setGraphList(JSON.stringify(responseData.graph_list, null, 2));
        setObstacleDetect(JSON.stringify(responseData.obstacle_detect, null, 4));
        setLidarSignal(JSON.stringify(responseData.lidar_signal, null, 4));
    }, [responseData]);

    return (
        <div className="response_callback_grid_container">
            <div className={`grid_item rbt_status ${rbtStatusExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleRbtStatus}>
                <div className={"grid_item_title"}>rbt_status</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{rbtStatus}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item service_status ${serviceStatusExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleServiceStatus}>
                <div className={"grid_item_title"}>service_status</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{serviceStatus}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item error_report ${errorReportExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleErrorReport}>
                <div className={"grid_item_title"}>error_report</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{errorReport}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item control_report ${controlReportExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleControlReport}>
                <div className={"grid_item_title"}>control_report</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{controlReport}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item graph_list ${graphListExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleGraphList}>
                <div className={"grid_item_title"}>graph_list</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{graphList}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item obstacle_detect ${obstacleDetectExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleObstacleDetect}>
                <div className={"grid_item_title"}>obstacle_detect</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{obstacleDetect}</pre>
                    </div>
                </div>
            </div>
            <div className={`grid_item lidar_signal ${lidarSignalExpanded ? 'expanded' : 'collapsed'}`} onClick={toggleLidarSignal}>
                <div className={"grid_item_title"}>lidar_signal</div>
                <div className="grid_item_data_container">
                    <div className="grid_item_data">
                        <pre>{lidarSignal}</pre>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default ResponseComponent;
