/**
 * Data logging support file for "online" mode.
 * This content is managed in https://github.com/lancaster-university/codal-microbit-v2/
 *
 * This code should compile on IE11 and run sufficiently to show the table and the IE11
 * support notice.
 */

function UserGraphError(message) {
  let instance = new Error(message);
  Object.setPrototypeOf(instance, Object.getPrototypeOf(this));
  if (Error.captureStackTrace) {
    Error.captureStackTrace(instance, UserGraphError);
  }
  return instance;
}

(function () {
  let isParentPage = !location.href.split("?")[1];
  if (isParentPage) {
    const base = Object.keys(window.dl).reduce(function (acc, curr) {
      acc[curr] = window.dl[curr];
      return acc;
    }, {});
    // Overrides that provide additional behaviour when online.
    const overrides = {
      update: alert.bind(
        null,
        "To see the latest data that changed after you opened this file, you must unplug your calliope mini and plug it back in."
      ),
      clear: alert.bind(
        null,
        "The log is cleared when you reflash your calliope mini. Your program can include code or blocks to clear the log when you choose."
      ),
      copy: function () {
        base.copy();
        let button = window.event.currentTarget;
        button.innerText = "Copied";
        setTimeout(function () {
          button.innerText = "Copy";
        }, 1000);
      },
      load: function () {
        let wrapper = document.querySelector("#w");
        let isIE = Boolean(document.documentMode);
        if (isIE) {
          const browserWarning = wrapper.appendChild(
            document.createElement("p")
          );
          browserWarning.id = "browser-warning";
          browserWarning.innerText =
            "Internet Explorer (IE) is not supported. Please use Google Chrome, Microsoft Edge, Mozilla Firefox or Safari.";
        }

        let header = document.body.insertBefore(
          document.createElement("header"),
          wrapper
        );
        header.outerHTML =
          '<header><div class="header-strip"></div><div class="header-contents"><a href="https://calliope.cc"><svg role="img" aria-labelledby="calliope-logo" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" viewBox="0 0 525.216 126.9"><path fill="#4a5261" d="M223.716 88.8h.2l5.6 17-11.1.1 5.3-17.1zm-4.9-11.1-14.3 42.2-3.5.5.1 5.8 16.5-.1-.1-5.8-3.4-.6 2.1-6.7 15.7-.1 2.2 6.6-3.3.6v5.8l16.5-.2v-5.8l-3.5-.5-15-41.9-10 .2zm38.1 6.1 5.1.9.4 34.7-5.2 1.1.1 5.8 36.4-.3-.2-13.6-7.4.1-.4 6.1-13.6.1-.4-34.1 5.2-1v-5.9l-5.2.1-9.7.1h-5.2zm46.7 0 5.1.9.3 34.7-5.1 1.1.1 5.8 36.4-.3-.2-13.6-7.4.1-.4 6.1-13.7.1-.3-34.1 5.2-1v-5.9l-5.2.1-9.7.1h-5.2zm67 36.5-5.2-.9-.3-34.8 5.2-1-.1-5.9-20 .2v5.9l5.2.9.3 34.7-5.1 1.1v5.8l20.1-.2zm154.3-29.5-.2-13.2-33 .3h-5.2l.1 5.9 5.1.9.3 34.7-5.1 1.1.1 5.8 38.2-.3-.1-13.3-7.4.1-.3 5.8-15.7.1-.1-13.8 16.4-.2-.1-7.4-16.4.1-.1-12.1 15.5-.2.5 5.8z"/><g transform="translate(-183.184 -204.1)"><title id="calliope-logo">calliope mini logo</title><defs><path id="a" d="M133.2 204.1h234.3v188.7H133.2z"/></defs><clipPath id="b"><use xlink:href="#a" width="100%" height="100%" overflow="visible"/></clipPath><path fill="#4a5261" d="M312.3 246.7H277v-33.8c0-3.4 2.8-6.2 6.2-6.2h22.9c3.4 0 6.2 2.8 6.2 6.2v33.8z" clip-path="url(#b)"/></g><path fill="#8096a1" d="M98.616 10.7h25.8v31.8h-25.8z"/><path fill="#855c33" d="M41.716 26v44.9l52.3.2V26.2c0-14.4-11.8-26.2-26.2-26.2-14.4 0-26.1 11.6-26.1 26"/><path fill="#26a6ab" d="M111.516 126.9h-84.1c-26.2 0-37.4-33.3-16.6-49.2l47-35.2h71.3v66.8c0 9.8-7.9 17.6-17.6 17.6"/><path fill="#42c9c9" d="m17.816 72.5 44.6-33.3-8 30.5z"/><path fill="#f7f5e8" d="m62.416 39.2 6.1 19.4h-15.8zm12.1 0 9.7 19.4h-15.7z"/><path fill="#bdd1cf" d="m84.216 58.6-1.7 11.1h-28.1l-1.7-11.1z"/><path fill="#f7f5e8" d="M102.816 126.9c-2.6-19.3-7.8-42.7-20.1-57.2h-28.2c-12.3 14.5-17.6 37.9-20.1 57.2h68.4z"/><path fill="#fc9" d="M70.716 39.7h-4.5c-4.7 0-8.5-3.8-8.5-8.5V16.7h21.5v14.5c0 4.7-3.8 8.5-8.5 8.5"/><path fill="#bdd1cf" d="M55.116 126.9V69.7c9.1 21.4 20.4 47.3 34.2 57.2h-34.2z"/><path fill="#fc9" d="m68.516 58.6 6-19.4h-12.1z"/><path fill="#42c9c9" d="M129.116 42.6v39.2l-30.5-39.2z"/><path fill="#4a5261" d="m190.016 93.4-7.2.1-1.1-6.4c-1-.9-2.2-1.7-3.6-2.2-1.5-.5-3.2-.8-5.1-.8-4.2 0-7.4 1.6-9.7 4.7-2.2 3.1-3.3 7.1-3.3 12v1.7c0 4.9 1.2 8.9 3.5 12 2.3 3.1 5.5 4.6 9.6 4.5 1.9 0 3.7-.3 5.2-.9 1.6-.6 2.8-1.3 3.7-2.3l.9-6.5 7.2-.1.1 9.6c-1.9 2.3-4.4 4.1-7.4 5.5-3 1.4-6.4 2.1-10.1 2.1-6.5.1-11.8-2.1-16-6.6-4.2-4.5-6.3-10.2-6.4-17.3v-1.6c-.1-7 1.9-12.8 6-17.4 4.1-4.6 9.4-6.9 15.9-6.9 3.7 0 7.1.6 10.2 2 3 1.3 5.5 3.1 7.5 5.3l.1 9.5zm225.5 7.5c0-5-1.1-9.1-3.2-12.2-2.1-3.1-5.2-4.6-9.3-4.6-4.1 0-7.1 1.6-9.1 4.7s-2.9 7.2-2.9 12.3v.8c0 5.1 1.1 9.2 3.2 12.3 2.1 3.1 5.1 4.6 9.2 4.6s7.2-1.6 9.2-4.8c2-3.1 3-7.2 3-12.3l-.1-.8zm9.7.7c.1 7.1-1.9 13-5.9 17.6-4 4.7-9.3 7-15.9 7.1-6.5.1-11.8-2.2-15.9-6.8s-6.1-10.4-6.2-17.5v-.7c-.1-7 1.9-12.9 5.9-17.6 4-4.7 9.2-7.1 15.8-7.1 6.6-.1 11.9 2.2 16 6.8s6.2 10.4 6.3 17.5l-.1.7zm25.5-.5 8.2-.1c2.7 0 4.7-.8 6.1-2.3 1.4-1.5 2-3.4 2-5.7 0-2.3-.7-4.2-2.1-5.7-1.4-1.5-3.5-2.2-6.2-2.2l-8.2.1.2 15.9zm8-23.4c5.5 0 9.9 1.3 13.2 4.1 3.2 2.8 4.9 6.5 4.9 11.1s-1.5 8.4-4.7 11.2c-3.2 2.8-7.5 4.3-13.1 4.3l-8.2.1.1 10.7 5.2.9v5.8l-20 .2-.1-5.8 5.1-1-.3-34.7-5.2-1-.1-5.9h23.2z"/></svg></a></div></header>';

        let info = document.createElement("p");
        info.id = "info";
        wrapper.appendChild(info);

        let buttonBar = document.querySelector(".bb");
        let graphButton = buttonBar.appendChild(
          document.createElement("button")
        );
        graphButton.id = "graphButton";

        window.dl.resetGraph();

        if (window.Promise) {
          graphButton.onclick = window.dl.graph;
          let graphLib = document.createElement("script");
          window.dl.graphLibPromise = new Promise(function (resolve, reject) {
            graphLib.onload = resolve;
            graphLib.onerror = reject;
          });
          graphLib.src =
            "https://microbit.org/dl/libs/plotly/plotly-scatter-6291fec6d1543144ba90.js";
          document.head.appendChild(graphLib);
        }

        base.load();

        // The base.load() function attaches the DAPLink version to the original object
        this.daplinkVersion = base.dapVer;
      },

      hasGraph: function () {
        return Boolean(document.querySelector("#graph"));
      },

      resetGraph: function () {
        document.querySelector("#info").innerHTML =
          "This is the data on your calliope mini. To analyse it and create your own graphs, transfer it to your computer. You can copy and paste your data, or download it as a CSV file which you can import into a spreadsheet or graphing tool. <a href='https://microbit.org/get-started/user-guide/data-logging/'>Learn more about calliope mini data logging</a>.";
        let graphButton = document.querySelector("#graphButton");
        graphButton.innerText = "Visual preview";

        let abortRow = document.querySelector("#abort-row");
        if (abortRow) {
          abortRow.id = null;
        }

        let graph = document.querySelector("#graph");
        if (graph) {
          graph.remove();
        }
      },

      graph: function () {
        function readData(table) {
          if (table.rows.length === 0) {
            throw new UserGraphError("No data to graph.");
          }
          const rows = table.rows;
          const headings = [].slice.call(table.rows[0].cells).map(function (r) {
            return r.innerText;
          });
          if (headings.length < 2) {
            throw new UserGraphError(
              "The visual preview requires two or more columns. Timestamps must be enabled."
            );
          }
          if (!/Time \(.+\)/.test(headings[0])) {
            throw new UserGraphError(
              "To show a visual preview, timestamps must be enabled when logging data."
            );
          }
          let x = [];
          let ys = headings.slice(1).map(function () {
            return [];
          });
          let abortRow;
          outer: for (let r = 1; r < rows.length; ++r) {
            let cells = rows[r].cells;
            if (cells.length === 0) {
              // Last row expected to be empty. Might be able to fix in offline code.
              continue;
            }
            if (cells.length !== headings.length) {
              // Stop processing if the data changes shape.
              abortRow = r;
              break;
            }
            for (let c = 0; c < cells.length; ++c) {
              let v = Number(cells[c].innerText);
              if (isNaN(v)) {
                abortRow = r;
                break outer;
              }
              if (c === 0) {
                if (x[x.length - 1] > v) {
                  // Time cannot go backwards.
                  abortRow = r;
                  break outer;
                }
                x.push(v);
              } else {
                ys[c - 1].push(v);
              }
            }
          }
          if (abortRow) {
            rows[abortRow].id = "abort-row";
          }

          const colors = [
            // calliope mini brand colors
            "rgb(0, 200, 0)",
            "rgb(62, 182, 253)",
            "rgb(205, 3, 101)",
            "rgb(231, 100, 92)",
            "rgb(108, 75, 193)",
            "rgb(123, 205, 194)",
          ];
          let nextColor = 0;
          let nextMarker = 0;
          return {
            xAxisLabel: headings[0],
            abortRow: abortRow,
            data: ys.map(function (y, index) {
              return {
                name: headings[index + 1],
                type: "scatter",
                mode: "lines+markers",
                x: x,
                y: y,
                line: {
                  color: colors[nextColor++ % colors.length],
                },
                marker: {
                  // There are more than this but they look increasingly odd.
                  symbol: nextMarker++ % 24,
                },
              };
            }),
          };
        }

        function toggleGraph() {
          if (window.dl.hasGraph()) {
            window.dl.resetGraph();
            return;
          }

          let graphHeight = Math.max(
            450,
            Math.round(window.innerHeight * 0.75)
          );
          let graphWrapper = document.createElement("div");
          graphWrapper.id = "graph";
          graphWrapper.style.height = graphHeight;
          graphWrapper.style.textAlign = "center";
          window.requestAnimationFrame(function () {
            let loading = graphWrapper.appendChild(document.createElement("p"));
            loading.style.marginTop = "20vh";
            loading.innerHTML = "Loading visual preview&mldr;";
            let info = document.querySelector("#info");
            info.insertAdjacentElement("afterend", graphWrapper);

            window.requestAnimationFrame(function () {
              try {
                let table = document.querySelector("table");
                let graphData = readData(table);
                let graphFadeContainer = graphWrapper.appendChild(
                  document.createElement("div")
                );
                if (graphData.abortRow) {
                  let warningElement = graphFadeContainer.appendChild(
                    document.createElement("p")
                  );
                  warningElement.id = "graph-warning";
                  warningElement.innerHTML =
                    "The graph only includes rows down to <a href='#abort-row'>row " +
                    (graphData.abortRow + 1) +
                    "</a>. The visual preview does not support repeated headers or non-numeric data. The first column must be a timestamp.";
                }
                let graphContainer = graphFadeContainer.appendChild(
                  document.createElement("div")
                );
                graphContainer.setAttribute("role", "img");
                graphContainer.setAttribute(
                  "aria-label",
                  "Autogenerated line graph of the data in the table below."
                );
                graphFadeContainer.style.opacity = 0;

                info.style.transition = "none";
                // We fade it in along with the graph.
                info.style.opacity = 0;
                info.innerHTML =
                  "This is a visual preview of the data on your calliope mini. To analyse it in more detail or create your own graphs, transfer it to your computer. You can copy and paste your data, or download it as a CSV file which you can import into a spreadsheet or graphing tool. <a href='https://microbit.org/get-started/user-guide/data-logging/'>Learn more about calliope mini data logging</a>.";

                Plotly.newPlot(
                  graphContainer,
                  graphData.data,
                  {
                    height: graphHeight,
                    xaxis: {
                      title: graphData.xAxisLabel,
                      showgrid: true,
                    },
                    yaxis: {
                      showgrid: true,
                    },
                  },
                  {
                    displaylogo: false,
                    responsive: true,
                    toImageButtonOptions: {
                      filename: "MY_DATA",
                    },
                    modeBarButtonsToRemove: [
                      "select2d",
                      "lasso2d",
                      "autoScale2d",
                    ],
                  }
                );
                graphContainer.on("plotly_afterplot", function () {
                  document.querySelector("#graphButton").innerText =
                    "Close visual preview";

                  // We originally set the height for the loading indicator but must
                  // remove it now as we may use more space for any error message.
                  graphWrapper.style.height = null;
                  loading.remove();

                  let transition = "opacity 600ms linear";
                  graphFadeContainer.style.transition = transition;
                  info.style.transition = transition;
                  graphFadeContainer.style.opacity = 1;
                  info.style.opacity = 1;
                });
              } catch (e) {
                window.dl.resetGraph();

                if (e instanceof UserGraphError) {
                  alert(e.message);
                } else {
                  console.error(e);
                  alert("Unknown error creating the graph.");
                }
              }
            });
          });
        }

        // Not on IE11.
        if (window.dl.graphLibPromise) {
          window.dl.graphLibPromise.then(toggleGraph);
        }
      },
    };
    Object.keys(overrides).forEach(function (k) {
      window.dl[k] = overrides[k];
    });
  }
})();
