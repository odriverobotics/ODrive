# odrive_gui

Python requirements: `pip install flask flask_socketio flask_cors odrive`

If the default odrive python package is not desired, the path to the modules can be passed as command line arguments.

example on windows 10: 
```
./odrive_gui_win.exe C:/Users/<you>/ODrive/tools C:/Users/<you>/ODrive/Firmware
```

The first argument is for your local version of odrivetool, the second is for fibre.

## Development and testing instructions

### Project setup
```
npm install
```

### Compiles and hot-reloads for development
```
npm run serve
```

### Lints and fixes files
```
npm run lint
```

### Serve electron version of GUI
```
npm run electron:serve
```

### Package electron app into executable
```
npm run electron:build
```

### Build electron app for all platforms
```
npm run electron:build -- -mwl
```

### Building for rpi and potentially other ARM platform devices

PhantomJS is required as a dependency, so it must be installed first:
```
sudo apt install phantomjs
```

After it is installed, `npm run electron:build` can be used to build the AppImage for ARM

### Customize configuration
See [Configuration Reference](https://cli.vuejs.org/config/).
