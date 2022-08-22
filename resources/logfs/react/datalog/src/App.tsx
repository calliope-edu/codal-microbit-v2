import React from 'react';
import logo from './logo.svg';
import './App.css';
import Header from './Header';
import DataLogTable from './DataLogTable';
import Plot from 'react-plotly.js';
import { Data } from 'plotly.js';
import { Map, Marker } from "pigeon-maps";
import LineGraphVisualisation from './LineGraphVisualisation';
import { throws } from 'assert';
import MapVisualisation from './MapVisualisation';
import Button from './Button';

export interface DataLog {
  [key: string]: string[]
}

export interface VisualisationType {
  name: string;
  availablityError: (log: DataLog) => string | null;
  generate: (log: DataLog) => JSX.Element;
}

export class UserGraphError extends Error {

}

function convertTableToLog(table: HTMLTableElement) {
  let log: DataLog = {};

  if (table.rows.length === 0) {
    return log;
  }

  const headings = Array.prototype.slice.call(table.rows[0].cells).map(header => header.innerText);

  headings.forEach(heading => log[heading] = []);

  let abortRow = 0;

  for (let row = 1; row < table.rows.length; row++) {
    const cells = table.rows[row].cells;

    if (cells.length === 0) {
      continue;
    }

    if (cells.length !== headings.length) {
      abortRow = row;
      break;
    }

    for (let cell = 0; cell < cells.length; cell++) {
      log[headings[cell]].push(cells[cell].innerText);
    }
  }

  return log;
}

type GraphData = {
  xAxisLabel: string;
  abortRow: number | null;
  data: Data[];
}

function isIE(): boolean {
  // @ts-ignore
  return window.document.documentMode;
}

let visualPreview: VisualisationType | null = null;

const log: DataLog = {
  "Time (s)": ["20", "40", "60", "80", "100"],
  Latitude: ["100", "50", "40", "49", "48"],
  Longitude: ["58", "48", "29", "49", "46"],
  Test: ["58", "62", "38", "38", "39"]
};

const visualisations: VisualisationType[] = [
  LineGraphVisualisation, MapVisualisation
];

function availableVisualisations() {
  return visualisations.filter(vis => !vis.availablityError(log));
}

interface AppState {
  visualisation: VisualisationType | null;
}

class App extends React.Component<{}, AppState> {

  constructor() {
    super({});

    this.state = {
      visualisation: null
    };
  }

  render() {
    const visualPreviews = availableVisualisations();

    return (<div className="app">
      <Header />
      <main>
        <h1>micro:bit data log</h1>
        <div className="buttons">
          <button onClick={this.download}>Download</button>
          <button onClick={this.download}>Copy</button>
          <button onClick={this.download}>Update data…</button>
          <button onClick={this.download}>Clear log…</button>
          <Button dropdown={visualPreviews.map(vis => vis.name)} onDropdownSelected={index => this.visualise(index)}>{this.state.visualisation ? "Close " + this.state.visualisation.name : (visualPreviews.length === 0 ? "No visual previews available" : visualPreviews[0].name)}</Button>
        </div>
        <p id="info">
          This is the data on your micro:bit. To analyse it and create your own graphs, transfer it to your computer. You can copy and paste your data, or download it as a CSV file which you can import into a spreadsheet or graphing tool. <a href="https://microbit.org/get-started/user-guide/data-logging/" target="_blank">Learn more about micro:bit data logging</a>.
        </p>
        <div id="data">
        {this.state.visualisation && this.state.visualisation.generate(log)}
        <DataLogTable log={log} />
        </div>
      </main>
    </div>)
      ;
  }

  download = () => {

  }
  
  copy = () => {
  
  }
  
  updateData = () => {
  
  }
  
  clearLog = () => {
  
  }
  
  visualise = (visIndex: number) => {
    this.setState({visualisation: availableVisualisations()[visIndex]});
  }
}

export default App;
