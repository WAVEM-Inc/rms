import { Link } from 'react-router-dom';
import "./TopComponent.css";

export default function TopComponents() {
    return (
        <div className={"top_components"}>
            <div className="top_title_container">
                <Link className={"top_dash_board_link"} to={"/"}>
                    <div id={"top_dash_board_link_title"}>KTP ViZ</div>
                </Link>
            </div>
            <div className="top_nav_bar_container">
                <Link className={"top_data_board_link"} to={"/data"}>
                    <div id={"top_data_board_link_title"}>Data Board</div>
                </Link>
            </div>
        </div>
    );
}