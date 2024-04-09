import "./RequestComponent.css";


interface RequestComponentProps {
    onControlGraphSyncClick: () => void;
    onControlMoveToDestClick: () => void;
    onControlMsCompleteClick: () => void;
    onMissionClick: () => void;
    onDetectedObjectClick: () => void;
    onErrorStatusClick: () => void;
    onObstacleStatusClick: () => void;
    onCooperativeStartClick: () => void;
    onCooperativeStopClick: () => void;
}

const RequestComponent: React.FC<RequestComponentProps> = ({
    onControlGraphSyncClick,
    onControlMoveToDestClick,
    onControlMsCompleteClick,
    onMissionClick,
    onDetectedObjectClick,
    onErrorStatusClick,
    onObstacleStatusClick,
    onCooperativeStartClick,
    onCooperativeStopClick
}: RequestComponentProps) => {
    return (
        <div className={"request_btn_container"}>
            <button className={"btn_request"} onClick={onControlGraphSyncClick}>Control - GraphSync</button>
            <button className={"btn_request"} onClick={onControlMoveToDestClick}>Control - MoveToDest</button>
            <button className={"btn_request"} onClick={onControlMsCompleteClick}>Control - MsComplete</button>
            <button className={"btn_request"} onClick={onMissionClick}>Mission</button>
            <button className={"btn_request"} onClick={onDetectedObjectClick}>DetectedObject</button>
            <button className={"btn_request"}  onClick={onErrorStatusClick}>Error Status</button>
            <button className={"btn_request"}  onClick={onObstacleStatusClick}>Obstacle Status</button>
            <button className={"btn_request btn_cooperative_start"} onClick={onCooperativeStartClick}>Cooperative Start</button>
            <button className={"btn_request btn_cooperative_stop"} onClick={onCooperativeStopClick}>Cooperative Stop</button>
        </div>
    );
};

export default RequestComponent;