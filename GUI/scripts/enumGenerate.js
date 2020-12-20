const fs = require('fs');
const path = require('path');


fs.readFile(path.resolve(__dirname,'..','..','tools','odrive','enums.py'), 'utf8', function(err, data) {
    if (err) throw err;
    let enumsString = data;
    let lines = enumsString.split(process.platform == 'win32' ? '\r\n' : '\n');
    let enums = {};
    for (const line of lines) {
        if (line != '' && line[0] != '#') {
            let name;
            let value;
            name = line.split('=')[0];
            value = line.split('=')[1];
            enums[name.trim()] = parseInt(value.trim());
        }
    }
    fs.writeFile(path.resolve(__dirname, '../src/assets/odriveEnums.json'), JSON.stringify(enums, null, 4), function(err) {
        if (err) throw err;
    });
    console.log('Wrote ODrive enums to GUI/src/assets/odriveEnums.json');
});
