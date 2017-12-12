
'use strict';

// Declare required variables
var dataRollx = 0;
var dataRolly = 0;
var dataRollz = 0;
var dataRollxArray = [];
var dataRollyArray = [];
var dataRollzArray = [];
var accuracy = 2;
var orderOfMag = (Math.PI/180);

//Connect to socket.io
var serverIP = ADD HOST NAME;
var socket = io.connect(serverIP + 'add the port number');
console.log('Connected to: ' + serverIP);

// Start reading IMU data
runSocket();

function runSocket() {
        socket.on('serial_update', function(data) {
            if (data.charAt(0) === 'O') {
                console.log(data);
                var dataArray = data.split(seperator);  //seperator is comma or whatever seperates the values we get

                // set x
                dataRollx = (dataArray[1] *= orderOfMag).toFixed(accuracy);
                
                // set y
                dataRolly = (dataArray[2] *= orderOfMag).toFixed(accuracy);

                // set z
                dataRollz = (dataArray[3] *= orderOfMag).toFixed(accuracy);

                console.log(dataRollx + "," + dataRolly + "," + dataRollz);
            }
        });
}
