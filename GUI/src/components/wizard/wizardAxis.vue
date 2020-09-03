<template>
  <div class="wizardAxis card">
    <div class="title">Which axis are you configuring?</div>
    <div class="choices">
      <div class="choice card">
        M0
      </div>
      <div class="choice card">
        M1
      </div>
      <div class="choice card">
        M0 and M1
      </div>
    </div>
  </div>
</template>

<script>
export default {
    name: "wizardAxis"
}
</script>

<style scoped>

.wizardAxis {
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