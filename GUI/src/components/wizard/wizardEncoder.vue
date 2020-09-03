<template>
  <div class="wizardEncoder card">
    <div class="title">What encoder are you using?</div>
    <div class="choices">
      <div class="choice card">
        CUI AMT102-V
      </div>
      <div class="choice card">
        Incremental
      </div>
      <div class="choice card">
        Incremental with index
      </div>
      <div class="choice card">
        Absolute
      </div>
      <div class="choice card">
        Hall Effect
      </div>
      <div class="choice card">
        Sin/Cos
      </div>
    </div>
  </div>
</template>

<script>
export default {
    name: "wizardEncoder",
}
</script>

<style scoped>
.wizardEncoder {
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