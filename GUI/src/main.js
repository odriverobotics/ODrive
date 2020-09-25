import Vue from 'vue'
import App from './App.vue'
import store from './store'
import Tooltip from 'vue-directive-tooltip';
import 'vue-directive-tooltip/dist/vueDirectiveTooltip.css';
import 'typeface-roboto/index.css';

Vue.config.productionTip = false
Vue.use(Tooltip);

new Vue({
  store,
  render: h => h(App)
}).$mount('#app')
