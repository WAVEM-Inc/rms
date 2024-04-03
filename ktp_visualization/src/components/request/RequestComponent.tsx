import "./RequestComponent.css";


interface RequestComponentProps {
    onControlGraphSyncClick: () => void;
    onControlMoveToDestClick: () => void;
    onControlMsCompleteClick: () => void;
    onMissionClick: () => void;
    onDetectedObjectClick: () => void;
}

const RequestComponent: React.FC<RequestComponentProps> = ({ 
    onControlGraphSyncClick,
    onControlMoveToDestClick,
    onControlMsCompleteClick,
    onMissionClick,
    onDetectedObjectClick
}: RequestComponentProps) => {
    return (
        <div className={"request_btn_container"}>
            <button className={"btn_request"} onClick={onControlGraphSyncClick}>Control - GraphSync</button>
            <button className={"btn_request"} onClick={onControlMoveToDestClick}>Control - MoveToDest</button>
            <button className={"btn_request"} onClick={onControlMsCompleteClick}>Control - MsComplete</button>
            <button className={"btn_request"} onClick={onMissionClick}>Mission</button>
            <button className={"btn_request"} onClick={onDetectedObjectClick}>DetectedObject</button>
        </div>
    );
};

export default RequestComponent;