// this file is used for configuring electron-builder

module.exports = {
    pluginOptions: {
        electronBuilder: {
            preload: 'src/preload.js',
            //nodeIntegration: true,
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
                            "target": "AppImage",
                        }
                    ]
                }
            },
            //mainProcessArgs: ['C:/Users/pajoh/Desktop/ODrive_work/ODrive/tools', 'C:/Users/pajoh/Desktop/ODrive_work/ODrive/Firmware']
        }
    }
}