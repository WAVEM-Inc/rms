#include "builder/response/control_report/control_report_builder.hxx"

ktp::build::ControlReportBuilder::ControlReportBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
}

ktp::build::ControlReportBuilder::~ControlReportBuilder()
{
    
}

ktp_data_msgs::msg::ControlReportDataGraphList ktp::build::ControlReportBuilder::build_data_graph_list()
{
    ktp_data_msgs::msg::ControlReportDataGraphList::UniquePtr graph_list = std::make_unique<ktp_data_msgs::msg::ControlReportDataGraphList>();

    // ############################################
    // map_id
    // 수신한 그래프 지도 아이디
    graph_list->set__map_id("asdfsaf");
    // ############################################

    // ############################################
    // version
    // 그래프 버전
    graph_list->set__version("gpahawsuigajsgb");
    // ############################################

    const ktp_data_msgs::msg::ControlReportDataGraphList &&graph_list_moved = std::move(*graph_list);

    return graph_list_moved;
}

ktp_data_msgs::msg::ControlReportData ktp::build::ControlReportBuilder::build_data()
{
    ktp_data_msgs::msg::ControlReportData::UniquePtr data = std::make_unique<ktp_data_msgs::msg::ControlReportData>();

    // ############################################
    // graph_list
    // 수신 그래프 리스트
    std::vector<ktp_data_msgs::msg::ControlReportDataGraphList> graph_list_vec;
    graph_list_vec.push_back(this->build_data_graph_list());
    data->set__graph_list(graph_list_vec);
    // ############################################

    const ktp_data_msgs::msg::ControlReportData &&data_moved = std::move(*data);

    return data_moved;
}

ktp_data_msgs::msg::ControlReport ktp::build::ControlReportBuilder::build_control_report()
{
    ktp_data_msgs::msg::ControlReport::UniquePtr control_report = std::make_unique<ktp_data_msgs::msg::ControlReport>();

    // ############################################
    // create_time
    // 생성 일시
    const std::string &create_time = get_current_time();
    control_report->set__create_time(create_time);
    // ############################################

    // ############################################
    // control_id
    // 제어/임무/그래프 전송 아이디
    control_report->set__control_id("zzsszzss");
    // ############################################

    // ############################################
    // control_type
    // 수신 타입
    control_report->set__control_type("control");
    // ############################################

    // ############################################
    // control_code
    // 수신 제어 코드 (control_type이 control일 경우에만 필요)
    control_report->set__control_code("qwerjgjiugbque wbjiahdsbgkj");
    // ############################################

    // ############################################
    // response_code
    // 정상 수신/실행 성공/실패 코드
    control_report->set__response_code(200);
    // ############################################

    // ############################################
    // data
    // 응답 상세 내용
    control_report->set__data(this->build_data());
    // ############################################

    const ktp_data_msgs::msg::ControlReport &&control_report_moved = std::move(*control_report);

    return control_report_moved;
}