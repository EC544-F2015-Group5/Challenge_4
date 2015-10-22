/*
    app.js
    EC544 Challenge 04 Group 05
*/

// Dependencies
var SerialPort = require("serialport");
var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Reads in port info from command line
var portName = process.argv[2];

// Sets port configuration
var portConfig = {
  baudRate: 9600,
  parser: SerialPort.parsers.readline("\n")
};

// Create a new SerialPort object
var sp = new SerialPort.SerialPort(portName, portConfig);

// Serve static assets such as CSS and JS files along with any request
app.use('/', express.static(__dirname + '/public'), function(req, res, next) {
  console.log("Client: Page loaded.");
});


// Set server to listen to HTTP on port 3000
http.listen(3000, function(){
  console.log('listening on *:3000.');
});

// Client -> Server (HTTP)
io.on('connection', function(socket) {
  // Console notification of connect/disconnect
  console.log('Server: User connected.');
  socket.on('disconnect', function() {
    console.log('Server: User left.');
  });
  socket.on('Start',function(){
    sp.write('Start\n');
    console.log("Crawler is in Motion!")
  });
  socket.on('Stop',function(){
    sp.write('Stop\n');
    console.log("Crawler has turned Off!")
  });
});
