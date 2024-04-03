import { Route, Switch } from 'react-router-dom';
import './App.css';
import DashboardPage from './page/DashBoardPage';

function App() {
  return (
    <div className="App">
      <Switch>
        <Route exact path={"/"}>
          <DashboardPage />
        </Route>
      </Switch>
    </div>
  );
}

export default App;
