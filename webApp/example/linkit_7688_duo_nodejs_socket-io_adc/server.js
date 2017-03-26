#!/usr/bin/env node

//////////////////////////////////////////////////////////////////////
// Author: rsp @KMUTNB
// Date: 2017-03-20
//////////////////////////////////////////////////////////////////////

var fs  = require("fs");
var url = require("url");
var SerialPort  = require("serialport").SerialPort;
// var SerialPort = require('serialport');
var http = require("http").createServer(handle);

var portName = "/dev/ttyUSB0";
var root = "web";

var buf = []; // global data buffer

function handle(req, res) {
  var request = url.parse(req.url, false);
  var filename = request.pathname;

  console.log("Serving request: " + request.pathname);

  if(filename == "/") { filename = "/index.html"; }

  filename = root + filename;

  try {
    fs.realpathSync(filename);
  } catch (e) {
    res.writeHead(404);
    res.end();
  }

  var contentType = "text/plain";

  if (filename.match(".js$")) {
    contentType = "text/javascript";
  } else if (filename.match(".html$")) {
    contentType = "text/html";
  }

  fs.readFile(filename, function(err, data) {
    if (err) {
      res.writeHead(500);
      res.end();
      return;
    }

    res.writeHead(200, {"Content-Type": contentType});
    res.write(data);
    res.end();
  });
}

http.listen( 8000 );

console.log( "server started on localhost:8000" );

// server listens for socket.io communication at port 8000
var io = require("socket.io").listen(http);

var ser = new SerialPort(portName, {
  baudrate: 921600
}); // instantiate the serial port.

ser.on("close", function (err) {
  console.log("port closed..");
});

ser.on("error", function (err) {
  console.error("error", err);
  // console.log("didi");
});

ser.on("open", function () {
  console.log("didi");
  console.log("port opened...");
});

io.sockets.on("connection", function (socket) {
    console.log('connected...');
    // ser.write('ARM\n');
    // console.log();
    // If socket.io receives message from the client browser then
    // this call back will be executed.
    socket.on("message", function (msg) {
        console.log('message: ' + msg);
        ser.write('ARM\n');
    });
    // If a web browser disconnects from Socket.IO then this callback is called.
    socket.on("disconnect", function () {
        console.log("disconnected");
        ser.write('DISARM\n');
    });
});

ser.on("data", function(data) { // call back when data is received
    data = data.toString().trim();
    //console.log("serial port: " + data);

    if ( data[0] === '#' ) {
       console.log( '> ' + data );
       if ( data === '#END' ) {
          buf.forEach( function(item) { // send data to the web client via socket.io
             var fields = item.split(','); // split input string into fields
             io.sockets.emit("message", fields[1] ); // send fields[1] only (for demo)
          });
          buf = [];
       }
    }
    else {
       buf.push( data );  // save data to buffer
    }
});

//////////////////////////////////////////////////////////////////////
