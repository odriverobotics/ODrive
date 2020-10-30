<template>
  <div class="cal-message"
    v-tooltip.top="{
            content: data.tooltip,
            class: 'tooltip-custom tooltip-other-custom fade-in',
            delay: 0,
            visible: !calStatus,
          }"
  >{{message}}</div>
</template>

<script>
export default {
  name: "wizardCalStatus",
  props: {
    data: Object,
    calStatus: Boolean,
  },
  computed: {
    message() {
      let retmsg = "";
      if (this.calStatus == true) {
        retmsg = this.data.success;
      } else if (this.calStatus == false) {
        retmsg = this.data.failure;
      }
      return retmsg;
    },
  },
};
</script>

<style scoped>
.cal-message {
    margin-top: auto;
    margin-bottom: auto;
}
</style>