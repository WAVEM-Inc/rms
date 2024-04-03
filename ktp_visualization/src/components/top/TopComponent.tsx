import { Link } from 'react-router-dom';
import "./TopComponent.css";

export default function TopComponents()
{

    return (
        <div className={"top_components"}>
            <Link className={"top_title_link"} to={"/"}>
                <div id={"top_title"}>KTP ViZ</div>
            </Link>
        </div>
    );
}