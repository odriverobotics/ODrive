// this file is used for configuring electron-builder

// https://github.com/webpack-contrib/copy-webpack-plugin/issues/29

const webpack = require('webpack')
//const CopyWebpackPlugin = require("copy-webpack-plugin");
//const WriteFilePlugin = require('write-file-webpack-plugin');
const path = require('path');

module.exports = {
    pluginOptions: {
        electronBuilder: {
            preload: 'src/preload.js',
            nodeIntegration: true,
            builderOptions: {
                "productName": "ODriveGUI",
                "asar": false,
                "extraResources": "server",
                "artifactName": "${name}.${ext}",
                "win" : {
                    "target" : [
                        {
                            "target": "portable",
                        }
                    ]
                },
                "linux" : {
                    "target" : [
                        {
                            "target": "electron-renderer",
                        }
                    ]
                }
            },
            //mainProcessArgs: ['C:/Users/pajoh/Desktop/ODrive_work/ODrive/tools', 'C:/Users/pajoh/Desktop/ODrive_work/ODrive/Firmware']
        }
    },
    configureWebpack: {
        module: {
          rules: [
            {
              test: /\.wasm$/,
              type: "javascript/auto"
            }
          ]
        },
        //externals: {
        //    usb: {
        //        commonjs: 'webusb',
        //        root: ['usb']
        //    }
        //},
        //target: 'electron-webpack',
        plugins: [
            //new CopyWebpackPlugin({patterns: [
            //    { from: path.join(__dirname, "node_modules", "usb", "build", "Release", "usb_bindings.node"),
            //    //to: path.join(__dirname, '[name].node')
            //},
            //  ]}),
            //  new WriteFilePlugin(),
            new webpack.NormalModuleReplacementPlugin(
              /^bindings$/,
              `${__dirname}/src/bindings`
            ),
          ],
        
    }
}