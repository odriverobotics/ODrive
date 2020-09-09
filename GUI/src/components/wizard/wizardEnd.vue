<template>
  <div class="card">
      <div class="diff" v-for="diff in configDiffs" :key="diff.path">
        <span class="diff-path">{{diff.path}} : </span>
        <span class="diff-old">{{diff.oldVal}}</span>
        <span class="diff-seperator"> ⮕ </span>
        <span class="diff-new">{{diff.newVal}}</span>
      </div>
  </div>
</template>

<script>
export default {
  name: "wizardEnd",
  props: {
    config: Object,
    axis: String,
    // actual odrive config is in this.$store (Vuex store)
  },
  data: function() {
    return {
      configDiffs: [], // array of objects {path, oldval, newval}
      flatpaths: [],
      path: [],
    }
  },
  created() {
    // flatten config tree into array of full variable paths
    this.pathFromTree(this.config);
    for (const path of this.flatpaths){
      console.log(path);
    }
    for (const path of this.flatpaths) {
      let odrvObj = this.$store.state.odrives.odrive0;
      let configObj = this.config;
      for (const key of path.split('.')){
        odrvObj = odrvObj[key];
        configObj = configObj[key];
      }
      if (configObj != null){
              this.configDiffs.push({path: path, oldVal: parseFloat(odrvObj["val"]), newVal: configObj});
      }

    }
  },
  methods: {
    pathFromTree(tree){
      // array path
      for (const key of Object.keys(tree)) {
        this.path.push(key);
        if (tree[key] != null && typeof tree[key] == "object") {
          this.pathFromTree(tree[key]);
        }
        else {
          this.flatpaths.push(this.path.join('.'));
        }
        this.path.pop();
      }
    }
  }
}
</script>

<style scoped>
.diff {
  text-align: left;
}

.diff-path {
    font-family: "Roboto Mono", monospace;
}

.diff-old {
  font-weight: bold;
}

.diff-new {
  font-weight: bold;
  color: green;
}

</style>