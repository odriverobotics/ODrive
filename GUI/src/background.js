'use strict'

import { app, protocol, BrowserWindow, ipcMain } from 'electron'
import { createProtocol } from 'vue-cli-plugin-electron-builder/lib'
import installExtension, { VUEJS_DEVTOOLS } from 'electron-devtools-installer'
const isDevelopment = process.env.NODE_ENV !== 'production'

const { spawnSync, execSync } = require('child_process');
const spawn = require('child_process').spawn;
const path = require('path');

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let win;

// var for python server
let server;

// Scheme must be registered before the app is ready
protocol.registerSchemesAsPrivileged([
  { scheme: 'app', privileges: { secure: true, standard: true } }
])

// function to get determine correct command for python
function getPyCmd() {
  // call both 'python' and 'python3' to figure out the correct command
  let spawnRet = spawnSync('python', ['-V']);
  let success = spawnRet.status != null;
  let outputString;
  let cmd = '';
  if (success) {
    if (spawnRet.stdout.toString().length > 1) {
      outputString = spawnRet.stdout.toString();
    }
    else {
      outputString = spawnRet.stderr.toString();
    }
    if (outputString.includes("Python 3")) {
      cmd = 'python';
    }
  }
  if (cmd == '') {
    spawnRet = spawnSync('python3', ['-V']);
    success = spawnRet.status != null;
    if (success) {
      if (spawnRet.stdout.toString().length > 1) {
        outputString = spawnRet.stdout.toString();
      }
      else {
        outputString = spawnRet.stderr.toString();
      }
      if (outputString.includes("Python 3")) {
        cmd = 'python3';
      }
    }
  }
  return cmd;
}

function createWindow() {

  // Create the browser window.
  win = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      // Use pluginOptions.nodeIntegration, leave this alone
      // See nklayman.github.io/vue-cli-plugin-electron-builder/guide/security.html#node-integration for more info
      nodeIntegration: process.env.ELECTRON_NODE_INTEGRATION,
      preload: path.join(__dirname, 'preload.js')
    },
    icon: path.join(__static, 'icon.png')
  })
  win.maximize();
  win.setMenu(null);

  if (process.env.WEBPACK_DEV_SERVER_URL) {
    // Load the url of the dev server if in development mode
    win.loadURL(process.env.WEBPACK_DEV_SERVER_URL)
    if (!process.env.IS_TEST) win.webContents.openDevTools()
  } else {
    createProtocol('app')
    // Load the index.html when not in development
    win.loadURL('app://./index.html')
  }

  win.on('closed', () => {
    win = null
  })
}

// Quit when all windows are closed.
app.on('window-all-closed', () => {
  // kill flask server
  if (process.platform !== 'win32') {
    execSync('kill $(ps aux | grep \'[o]drive_server.py\' | awk \'{print $2}\')');
    console.log("killed flask server");
  }
  // On macOS it is common for applications and their menu bar
  // to stay active until the user quits explicitly with Cmd + Q
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  // On macOS it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  if (win === null) {
    createWindow()
  }
})

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on('ready', async () => {
  if (isDevelopment && !process.env.IS_TEST) {
    // Install Vue Devtools
    try {
      await installExtension(VUEJS_DEVTOOLS)
    } catch (e) {
      console.error('Vue Devtools failed to install:', e.toString())
    }
  }
  createWindow()
  // Figure out correct path and arguments to launch python server
  let scriptFilename = path.join(app.getAppPath(), '../server', 'odrive_server.py');
  const args = process.argv;
  let effectiveCommand = [];
  effectiveCommand.push(scriptFilename);
  if (app.isPackaged === true) {
    for (const arg of args.slice(1)) {
      effectiveCommand.push(arg);
    }
  }
  else {
    for (const arg of args.slice(2)) {
      effectiveCommand.push(arg);
    }
  }
  // launch python server on event from renderer process (gui) and pipe stdout/stderr to it
  ipcMain.on('start-server', () => {
    server = spawn(getPyCmd(), effectiveCommand);
    server.stdout.on('data', function (data) {
      try {
        console.log(data.toString('utf8'));
      } catch (error) {
        console.log(error);
      }
      try {
        win.webContents.send('server-stdout', String(data.toString('utf8')));
      } catch (error) {
        console.log(error);
      }
    });
    server.stderr.on('data', function (data) {
      console.log(data.toString('utf8'));
      try {
        win.webContents.send('server-stderr', String(data.toString('utf8')));
      } catch (error) {
        console.log(error);
      }
    });
  })
  ipcMain.on('kill-server', () => {
    server.kill('SIGINT');
  })
})

// Exit cleanly on request from parent process in development mode.
if (isDevelopment) {
  if (process.platform === 'win32') {
    process.on('message', (data) => {
      if (data === 'graceful-exit') {
        server.kill('SIGINT');
        app.quit()
      }
    })
  } else {
    process.on('SIGTERM', () => {
      server.kill('SIGINT');
      app.quit()
    })
  }
}
