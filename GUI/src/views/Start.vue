<template>
  <div class="home">
    <div class="logo">
      <img alt="Odrive Logo" src="../assets/odrive_logo.png">
    </div>
    <div class="home_text">
      To set up your ODrive, connect it and power it up.
    </div>
    <div class="connected-container">
      <input type="text" :class="{ notConnected: notConnected, connected: connected}" v-on:change="setUrl" :value="serverAddress">
    </div>
    <button class="show-msg-button" @click="showServerMessages = !showServerMessages;">Having trouble? Click here for debug output</button>
    <div class="server-msgs" v-if="showServerMessages">
      <div class="server-msg" v-for="(msg, index) in serverMessages" :key="msg + index">{{msg}}</div>
    </div>
  </div>
</template>

<script>

export default {
  name: 'Home',
  components: {
  },
  data() {
    return {
      showServerMessages: false,
    }
  },
  computed: {
    connected() {
      return this.$store.state.serverConnected == true;
    },
    notConnected() {
      return this.$store.state.serverConnected != true;
    },
    serverAddress() {
      return this.$store.state.odriveServerAddress;
    },
    serverMessages() {
      if (this.$store.state.serverOutput.length >= 10) {
        let len = this.$store.state.serverOutput.length;
        return this.$store.state.serverOutput.slice(len-10,len);
      }
      else {
        return this.$store.state.serverOutput;
      }
    }
  },
  methods: {
    setUrl(e) {
      console.log(e.target.value);
      this.$store.dispatch("setServerAddress", e.target.value);
    },
  }
}
</script>

<style>
.home {
  padding: 10% 0;
  margin: auto;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background-color: var(--bg-color);
  height: 100vh;
}

img {
  margin: auto;
  display: block;
  max-width: 50%;
  width: auto;
  height: auto;
}

.home_text {
  font-size: 1.5rem;
  padding: 20px;
}

.connected, .notConnected {
  width: 40vw;
  margin: 0;
  padding: 10px;
  border: none;
  border-bottom: 1px solid lightgrey;
  font-size: 1.5rem;
  background-color: var(--bg-color);
  text-align: center;
}

input:focus {
  outline: none;
}

.connected {
  border-bottom: 1px solid lightgreen;
  background-color: lightgreen;
}

.notConnected {
  border-bottom: 1px solid red;
}

.show-msg-button {
  border: none;
  background-color: var(--bg-color);
  margin-top: 0.5rem;
  cursor: pointer;
}

.server-msg {
  font-family: "Roboto Mono", monospace;
}
</style>
