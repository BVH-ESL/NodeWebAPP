#!/usr/bin/env node

//////////////////////////////////////////////////////////////////////
// Author: rsp @KMUTNB
// Date: 2017-03-20
//////////////////////////////////////////////////////////////////////

var fs  = require("fs");
var url = require("url");
var http = require("http").createServer(handle);

// var portName = "/dev/ttyS0";
var root = "web";
var buf = []; // global data buffer
var listUser = [];

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

io.sockets.on("connection", function (socket) {
    // If socket.io receives message from the client browser then
    // this call back will be executed.
    socket.on("message", function (msg) {
      listUser.push({
          socketID:   socket.id,
          nodeID: 0,
          isConnect: false
      });
    });

    // If a web browser disconnects from Socket.IO then this callback is called.
    socket.on("disconnect", function () {
        console.log("disconnected");
        for(var i in listUser){
          if(listUser[i].socketID === socket.id){
            listUser.splice(i, 1);
                break;
            }
        }
    });

    socket.on("cmd", function(msg){
      if (msg.split(',')[0] === "check") {
        var isNodeUsed = false;
        var socketIndex = -1;
        for(var i in listUser){
          if(listUser[i].nodeID === parseInt(msg.split(',')[1])){
            isNodeUsed = true;
          }
          if(listUser[i].socketID === socket.id){
            socketIndex = i;
          }
        }
        if(isNodeUsed){
          io.to(socket.id).emit("error", "This nodeID is allready use");
        }else{
          listUser[socketIndex].nodeID = parseInt(msg.split(',')[1]);
          client.publish("esl/"+msg.split(',')[1]+"/in/cmd", "CHECK");
          setTimeout(function(){
            if(listUser[socketIndex].isConnect){
              io.to(socket.id).emit("status", "#CONNECTED");
            }else{
              listUser[socketIndex].nodeID = 0;
              io.to(socket.id).emit("error", "Monitor Node ที่คุณเลือกไม่ได้เปิดใช้งาน \n กรุณาเลือก Monitor node ใหม่");
            }
          }, 1000);
        }
        // client.publish("esl/"+msg.split(',')[1]+"/in/cmd", msg);
      }else if(msg.split(',')[0] === "SET"){
        for(var i in listUser){
          if(listUser[i].socketID === socket.id){
            if(listUser[i].isConnect === false){
              io.to(socket.id).emit("error", "ยังไม่ไได้เชื่อมต่อกับ Monitor node \n กรุณาเชื่อมต่อ");
            }else{
              // client.publish("esl/"+listUser[i].nodeID+"/in/cmd", msg);
              if(msg.split(',')[1] === "time"){
                console.log(msg);
                client.publish("esl/"+listUser[i].nodeID+"/in/cmd", msg);
              }else if(msg.split(',')[1] === "count"){
                client.publish("esl/"+listUser[i].nodeID+"/in/cmd", msg);
              }
            }
            break;
          }
        }
      }else if(msg.split(',')[0] === "ARM" || msg.split(',')[0] === "DISARM"){
        // check arm and disarm
        for(var i in listUser){
          if(listUser[i].socketID === socket.id){
            if(listUser[i].isConnect === false){
              io.to(socket.id).emit("error", "ยังไม่ไได้เชื่อมต่อกับ Monitor node \n กรุณาเชื่อมต่อ");
            }else{
              client.publish("esl/"+listUser[i].nodeID+"/in/cmd", msg);
            }
            break;
          }
        }
      }
      // client.publish("esl/1/in/cmd", msg);
    });
});

var mqtt = require('mqtt');
var client  = mqtt.connect('mqtt://192.168.1.9');

client.on('connect', function () {
  console.log("MQTT Connect");
  client.subscribe('esl/+/out/+');
});

client.on('message', function (topic, message) {
  // message is Buffer
  // console.log(topic.toString());
  // client.end()
  var data = message.toString();
  // console.log(data);
  if(topic.toString().split('/')[3] === "status"){
    for(var i in listUser){
      if(listUser[i].nodeID === parseInt(topic.toString().split('/')[1])){
        // console.log("found");
        socketIndex = i;
        console.log(socketIndex);
        break;
      }
    }
    io.to(listUser[socketIndex].socketID).emit("status", data);
    // console.log(message.toString());
    var data = message.toString();
    if(data === "#END"){
      var socketIndex = 0;
        // console.log(JSON.parse(buf[0]));
        // obj = JSON.parse(str);
      buf.forEach( function(item) { // send data to the web client via socket.io
        //  var fields = item.split(','); // split input string into fields
         io.to(listUser[socketIndex].socketID).emit("message", item ); // send fields[1] only (for demo)
      });
      buf = [];
    }else if(data === "#ACK"){
      for(var i in listUser){
        if(listUser[i].nodeID === parseInt(topic.toString().split('/')[1])){
          listUser[i].isConnect = true;
          // console.log("found");
          break;
        }
      }
    }
  }else if(topic.toString().split('/')[3] === "data"){
    // var data = message.toString().split(",");
    // console.log(data);
    buf.push(data);
  }
})


// ser.on("data", function(data) { // call back when data is received
//     data = data.toString().trim();
//     //console.log("serial port: " + data);
//
//     if ( data[0] === '#' ) {
//        console.log( '> ' + data );
//        if ( data === '#END' ) {
//           buf.forEach( function(item) { // send data to the web client via socket.io
//              var fields = item.split(','); // split input string into fields
//              io.sockets.emit("message", fields[1] ); // send fields[1] only (for demo)
//           });
//           buf = [];
//        }
//     }
//     else {
//        buf.push( data );  // save data to buffer
//     }
// });

//////////////////////////////////////////////////////////////////////
