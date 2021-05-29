<template>
  <div class="card plot">
    <div class="plot-header">
      <button class="close-button" @click="deletePlot">X</button>
      <button class="close-button" @click="exportCSV">Export</button>
      <button class="close-button" @click="$emit('add-var', name)">+</button>
    </div>
    <line-chart v-if="loaded" :chart-data="datacollection" :options="dataOptions"></line-chart>
  </div>
</template>

<script>
import LineChart from "./LineChart.js";
import { saveAs } from "file-saver";

export default {
  components: {
    LineChart,
  },
  props: ["name", "plot", "dashID"],
  data() {
    return {
      datacollection: null,
      timeStart: null,
      loaded: false,
      dataOptions: {
        animation: {
          duration: 0, // general animation time
        },
        events: [],
        hover: {
          animationDuration: 0, // duration of animations when hovering an item
        },
        responsiveAnimationDuration: 0, // animation duration after a resize
        elements: {
          point: {
            radius: 0,
          },
          line: {
            //  borderColor: "rbga(0,0,0,0)",
            fill: false,
            borderWidth: 0,
            tension: 0,
          },
        },
        responsive: true,
        maintainAspectRatio: false,
      },
    };
  },
  mounted() {
    this.timeStart = Date.now();
    //this.initData(); //set label for dataset and color
    this.cyclicUpdate = setInterval(() => this.fillData(), 50);
  },
  beforeDestroy() {
    clearInterval(this.cyclicUpdate);
  },
  methods: {
    fillData() {
      let newData = {
        labels: this.$store.state.propSamples["time"],
        datasets: [],
      };
      for (const plotVar of this.plot.vars) {
        let newPath = plotVar.path.split(".");
        newPath.splice(0, 1);
        newPath = newPath.join(".");
        newData.datasets.push({
          label: newPath,
          borderColor: plotVar.color, //"rgba(0,0,0,0)",
          data: this.$store.state.propSamples[newPath],
        });
      }
      this.datacollection = newData;
      this.loaded = true;
    },
    initData() {
      this.datacollection = {
        labels: [0],
        datasets: [
          {
            label: "Sine wave",
            backgroundColor: "rgba(0,0,0,0)", //"#f87979",
            data: [0],
          },
        ],
      };
    },
    deletePlot: function () {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removePlotFromDash", {
        dashID: this.dashID,
        plotID: this.name,
      });
    },
    exportCSV: function () {
      // make a sensible data structure
      let csvData = {};
      csvData["time"] = this.$store.state.propSamples["time"];
      for (const dataset of this.datacollection.datasets) {
        csvData[dataset.label] = dataset.data;
      }
      let csvString = "";
      let dataKeys = Object.keys(csvData);
      // turn data structure into a string that's suitable for exporting
      for (const label of dataKeys) {
        csvString += label;
        if (label != dataKeys.slice(-1)) {
          csvString += ",";
        } else {
          csvString += "\n";
        }
      }
      for (let idx = 0; idx < csvData["time"].length; idx = idx + 1) {
        for (const label of dataKeys) {
          csvString += csvData[label][idx];
          if (label != dataKeys.slice(-1)) {
            csvString += ",";
          } else {
            csvString += "\n";
          }
        }
      }
      var blob = new Blob([csvString], { type: "text/plain;charset=utf-8" });
      saveAs(blob, "plot.csv");

      console.log("exporting plot");
    },
  },
};
</script>

<style scoped>
div {
  position: relative;
}

.plot {
  z-index: 0;
}

.plot-header {
  display: flex;
}

.plotname {
  flex-grow: 10;
  margin: auto 0;
}

.delete {
  font-weight: bold;
  cursor: pointer;
  padding: 0 5px;
  margin-right: 10px;
  border: 1px solid black;
}

.add-var {
  font-weight: bold;
  cursor: pointer;
  padding: 0 5px;
  margin-right: 10px;
  border: 1px solid black;
}
</style>