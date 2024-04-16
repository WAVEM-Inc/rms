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
    onCanEmergencyClick: () => void;
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
    onCooperativeStopClick,
    onCanEmergencyClick
}: RequestComponentProps) => {
    return (
        <div className={"request_btn_container"}>
            <button className={"btn_request"} onClick={onControlGraphSyncClick}>그래프</button>
            <button className={"btn_request"} onClick={onControlMoveToDestClick}>제어(MoveToDest)</button>
            <button className={"btn_request"} onClick={onControlMsCompleteClick}>제어(MsComplete)</button>
            <button className={"btn_request"} onClick={onMissionClick}>임무</button>
            <button className={"btn_request"} onClick={onDetectedObjectClick}>가상 장애물 감지</button>
            <button className={"btn_request"}  onClick={onErrorStatusClick}>가상 에러</button>
            <button className={"btn_request"}  onClick={onObstacleStatusClick}>가상 장애물 정보</button>
            <button className={"btn_request btn_cooperative_start"} onClick={onCooperativeStartClick}>협동 주행 시작(LiDAR)</button>
            <button className={"btn_request btn_cooperative_stop"} onClick={onCooperativeStopClick}>협동 주행 중지(LiDAR)</button>
            <button className={"btn_request"} onClick={onCanEmergencyClick}>비상 정지</button>
        </div>
    );
};

export default RequestComponent;