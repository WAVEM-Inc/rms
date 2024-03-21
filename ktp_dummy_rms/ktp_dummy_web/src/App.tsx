import React from 'react';
import './App.css';
import {Route, Switch} from "react-router";
import DashBoardPage from "./page/dashboard/DashBoardPage";

function App() {
  return (
    <div className="App">
        <Switch>
            <Route exact path={"/"} component={DashBoardPage} />
        </Switch>
    </div>
  );
}

export default App;
