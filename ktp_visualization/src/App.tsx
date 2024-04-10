import { Route, Switch } from 'react-router-dom';
import './App.css';
import DashBoardPage from './page/dashboard/DashBoardPage';
import DataBoardPage from './page/databoard/DataBoardPage';

function App() {
  return (
    <div className="App">
      <Switch>
        <Route exact path={"/"}>
          <DashBoardPage />
        </Route>
        <Route exact path={"/data"}>
          <DataBoardPage />
        </Route>
      </Switch>
    </div>
  );
}

export default App;
