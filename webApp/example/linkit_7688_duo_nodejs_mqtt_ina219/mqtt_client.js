#!/usr/bin/env node

// Date: 2017-03-22
// Test commands
// Send command 'ARM' to the MQTT broker
// $  mosquitto_pub -h 192.168.1.9 -p 1883 -t 'esl/1234/in/cmd' -m 'ARM'
// Subscribe messages on the specified topic with the MQTT broker
// $  mosquitto_sub -h 192.168.1.9 -p 1883 -t 'esl/+/out/#'

var serialport = require('serialport');
var mqtt = require('mqtt');
var fs = require('fs');
var jsonfile = require('jsonfile');

var data_count = 0;
var data_buf = [];
var data_recording = false;

var topic_prefix = 'esl/1234';
var topic_cmd    = topic_prefix + '/in/cmd';
var topic_status = topic_prefix + '/out/status';
var topic_data   = topic_prefix + '/out/data';

//---------------------------------------------------------------
var broker_host = '192.168.1.9';
var broker_port = 1883;

var mqtt_client = mqtt.connect( {
    host:broker_host, port:broker_port, keepalive: 20000
});

//---------------------------------------------------------------
var portName = '/dev/ttyS0';
var SerialPort = serialport.SerialPort;
var ser = new SerialPort(portName, {
    baudRate: 500000,
    dataBits: 8,
    parity: 'none',
    stopBits: 1,
    flowControl: false,
    parser: serialport.parsers.readline("\n")
});

ser.on('error', function(err) {
    console.log( 'serial error: ' + err );
});

mqtt_client.on('connect', function () {
    console.log('MQTT connected...');
    this.subscribe( topic_cmd );
    this.publish( topic_status, '#READY' );
});

mqtt_client.on('error', function(err) {
    console.log('MQTT error: ', err);
});

mqtt_client.on('message', function(topic, message) {
   var msg = message.toString().trim();
   if ( topic === topic_cmd ) {
      if ( msg === 'ARM' ) {
         ser.write( msg + '\n' );
         mqtt_client.publish( topic_status, "#ARMED", { qos: 1 } );
         console.log('MQTT command received:' + msg );
      }
      else if ( msg === 'DISARM' ) {
         ser.write( msg + '\n' );
         mqtt_client.publish( topic_status, "#DISARMED", { qos: 1 } );
         console.log('MQTT command received:' + msg );
      }
      else if ( msg === 'CHECK' ) {
         mqtt_client.publish( topic_status, "#ACK", { qos: 1 } );
         console.log('MQTT command received:' + msg );
      }
      else {
         console.log('MQTT error: unknown command... ' + msg );
      }
   } else {
      console.log('MQTT error: topic not supported...');
   }
});

ser.on('data', function(data) {
    data = data.toString().trim();

    if ( data[0] === '#' ) {
       console.log( data );
       if ( data === '#START' ) {
          data_recording = true;
          data_buf = [];
          data_count = 0;
       } else if ( data === '#END' ) {
          console.log( 'count: ' + data_count );
          data_recording = false;
          save_jsonfile( 'data.json', data_buf );
       } else if ( data === '#READY' ) {
          mqtt_publish_all_data( data_buf );
       }
       mqtt_client.publish( topic_status, data, { qos: 1 } );
       return;
    }

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
         'mA':mA.toFixed(1),
         'mW':((mV*mA)/1000).toFixed(1),
         'st':st,
         'us':us
        } );
       data_count++;
    }
});

ser.on('open', function() {
    console.log( 'serial open..' );
} );

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

function mqtt_publish_all_data( data ) {
    data.forEach( function(item) {
       mqtt_client.publish( topic_data, JSON.stringify(item), { qos: 1 } );
    });
}

console.log('Program started...');

///////////////////////////////////////////////////////////////////////////////////////
