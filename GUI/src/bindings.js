// Source: https://stackoverflow.com/a/50547650/3621512

/* eslint-disable */

module.exports = x => {
    console.log("load...", `${require("electron").remote.app.getAppPath()}/${x}`);
    __non_webpack_require__(
      `${require("electron").remote.app.getAppPath()}/${x}`
    );
}
