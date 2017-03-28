#!/usr/bin/env node

//////////////////////////////////////////////////////////////////////
// Author: rsp @KMUTNB
// Date: 2017-03-20
//////////////////////////////////////////////////////////////////////

var fs  = require("fs");
var url = require("url");
var SerialPort  = require("serialport").SerialPort;
var http = require("http").createServer(handle);
var jsonfile = require('jsonfile');

var data_count = 0;
var data_buf = [];
var data_recording = false;

var portName = "/dev/ttyS0";
var root = "web";
var webUser = "";

var nodeID = 2;

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
  baudrate: 500000
}); // instantiate the serial port.

ser.on("close", function (err) {
  console.log("port closed..");
});

ser.on("error", function (err) {
  console.error("error", err);
});

ser.on("open", function () {
  console.log("port opened...");
});

io.sockets.on("connection", function (socket) {
    console.log('connected...');
    socket.on("message", function (msg) {
        if(webUser === ""){
          webUser = socket.id;
        }else{
          io.to(socket.id).emit("error", "มี User ใช้งาน Sensor Node นี้อยู่ \n ขออภัยในความไม่สะดวก");
        }
    });
    // If a web browser disconnects from Socket.IO then this callback is called.
    socket.on("disconnect", function () {
        console.log("disconnected");
        if(webUser != ""){
          webUser = "";
        }
    });
    socket.on("cmd", function(msg){
      console.log(msg);
      if(socket.id === webUser){
        ser.write(msg+'\n');
      }else{
        io.to(socket.id).emit("error", "มี User ใช้งาน Sensor Node นี้อยู่ \n ขออภัยในความไม่สะดวก");
      }
    });
  });

ser.on("data", function(data) { // call back when data is received
    data = data.toString().trim();
    // console.log("serial port: " + data);
    if ( data[0] === '#' ) {
      if(webUser != ""){
          console.log("serial port: " + data);
         io.to(webUser).emit("status", data);
         if ( data === '#START' ) {
            data_recording = true;
            data_buf = [];
            data_count = 0;
         } else if ( data === '#END' ) {
            console.log( 'count: ' + data_count );
            data_recording = false;
            save_jsonfile( 'data.json', data_buf );
            emitData( data_buf );
         } else if ( data === '#READY' ) {
            // emitData( data_buf );
         }
     }
   }else{
    if ( data_recording ) {
       fields = data.split(',');
       var ct = parseInt(fields[0]);
       var mV = parseInt(fields[1]);
       var mA = parseFloat(fields[2]);
       var st = parseInt(fields[3]);
       var us = parseInt(fields[4]);
       data_buf.push( {
         'ct':ct,
         'mV':mV,
         'mA':parseFloat(mA.toFixed(1)),
         'mW':parseFloat(((mV*mA)/1000).toFixed(1)),
         'st':st,
         'us':us
        } );
       data_count++;
    }
  }
});

function save_jsonfile( filename, data ) {
    jsonfile.writeFile( filename, data, {spaces:1}, function(err){
       if (err != null) {
          console.error(err);
          return;
       }
       console.log('saved data to JSON file done..' );
    });
    console.log( data[0] ); // show the first data
    console.log( data[data.length-1] ); // show the last data
    //setTimeout( function(){ process.exit(); }, 1000 );
}

function emitData( data ) {
  // console.log( data[0] ); // show the first data
  // console.log( data[data.length-1] ); // show the last data
    data.forEach( function(item) {
       io.to(webUser).emit("message", item );
    });
}

//////////////////////////////////////////////////////////////////////
