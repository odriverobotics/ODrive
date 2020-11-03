<template>
  <div class="card">
    <button class="close-button" @click=deleteCtrl>X</button>
    <span class="ctrlName">{{name}}:</span>
    <div class="right">
      <input
        class="ctrlInput"
        v-if="writeAccess"
        type="checkbox"
        :id="'ctrlInput'+id"
        :value="checkboxValue"
        :checked="checkboxValue"
        @click="putVal"
        style="margin-top:3px; margin-right:5px;"
      />
      <label :for="'ctrlInput'+id" v-if="writeAccess" class="booleanValue">{{checkboxValue | capitalize}}</label>

      <button v-if="writeAccess" v-on:click="putVal" style="padding:0; margin-right:10px; margin-left:10px;" tooltip="Write down">
        <img src="../../assets/images/saveIcon.png"  />
      </button>
      <span class="ctrlVal booleanValue true"  v-if="value"  >{{ value | capitalize}}</span>
      <span class="ctrlVal booleanValue false" v-if="!value" >{{ value | capitalize }}</span>
    </div>
  </div>
</template>

<script>
import { getVal, getReadonly, putVal, fetchParam } from "../../lib/odrive_utils.js";
import { v4 as uuidv4 } from 'uuid';

export default {
  name: "CtrlBoolean",
  //type checking here for properties
  props: {
    path: String,
    odrives: Object,
    dashID: String,
    parentControl: Object
  },
  data() {
    return {
      id: uuidv4(),
      checkboxValue:false
    }
  },
  filters: {
    capitalize: function(val) {
      if (val === undefined) return ''
      val = val.toString()
      return val.charAt(0).toUpperCase() + val.slice(1)
    }
  },
  computed: {
    value: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getVal(keys.join('.'));
    },
    name: function () {
      let keys = this.path.split(".");
      keys.shift();
      return keys.join(".");
    },
    writeAccess: function () {
      let keys = this.path.split(".");
      keys.shift(); // don't need first key here
      return getReadonly(keys.join('.')) == false;
    },
  },
  methods: {
    putVal: function (e) {
      var isChecked = e.target.checked;
      if (isChecked === undefined) {
        isChecked = this.checkboxValue;
      }

      putVal(this.name, isChecked);
      fetchParam(this.name);
      this.checkboxValue = isChecked;
      this.parentControl.checkboxValue = isChecked;
    },
    deleteCtrl: function() {
      // commit a mutation in the store with the relevant information
      this.$store.commit("removeCtrlFromDash", {dashID: this.dashID, path: this.path});
    }
  },
  created() {
    // update parameter value on component creation
    let keys = this.path.split('.');
    keys.shift();
    fetchParam(keys.join('.'));
  
    if (this.parentControl.checkboxValue !== undefined) {
      this.checkboxValue = this.parentControl.checkboxValue;
    } else {
      this.checkboxValue = this.value;
    }
  }
};
</script>

<style scoped>
.ctrlVal {
  font-weight: bold;
}

.ctrlInput {
  margin-left: 10px;
}

.right {
  display: flex;
  flex-direction: row;
  margin-left: auto;
}

.card {
  display: flex;
}

.booleanValue {
  width:40px;
}

.true {
    color:green;
}

.false {
    color:red;
}
</style>