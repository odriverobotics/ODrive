<template>
  <div class="wizardMotor card">
    <div class="title">What motor are you using?</div>
    <div class="choices">
      <div class="choice card">
        ODrive D5065
      </div>
      <div class="choice card">
        ODrive D6374
      </div>
      <div class="choice card">
        Other
      </div>
    </div>
  </div>
</template>

<script>
export default {
    name: "wizardMotor",
}
</script>

<style scoped>
.wizardMotor {
  display: flex;
  flex-direction: column;
  text-align: center;
}

.choices {
  display: flex;
  flex-direction: row;
}

.choice {
  text-align: center;
  display: flex;
  flex-direction: column;
  margin: 2rem;
}

.chosen {
  border: 2px solid black;
}

.unchosen {
  border: 2px solid transparent;
}

img {
  padding: 0px;
  margin: auto;
}

</style>