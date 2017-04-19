var fs = require('fs');
var dgram = require('dgram');
var udpSocket = dgram.createSocket('udp4');
var moment = require('moment');
var port = 5550;

var _buffer = [];
var _bufferTimer = null;
var leakyDelay = 2500;

var imageCounter = 0;


var savePhoto = function() {
	//open file
	console.log("saving image");
	var data = Buffer.concat(_buffer);
	var filename = "images/image" + imageCounter++ + ".jpg";

	console.log("writing " + data.length + " bytes to file");
	fs.writeFileSync(filename, data);

	_buffer = [];
}
var leakyBufferFn = function(part) {
	if (_bufferTimer) {
		clearTimeout(_bufferTimer);
	}
	_buffer.push(part);
	_bufferTimer = setTimeout(savePhoto, leakyDelay);
}


udpSocket.on('listening', function() {
	console.log('Listening for UDP packets on port ' + port + ' ...');
});

udpSocket.on('message', function(msg, rinfo) {
	if (rinfo.size == 0) {
		return;
	}
	console.log("rinfo was ", rinfo);
	console.log('received [' + rinfo.address + '] (' + msg.length + ' bytes) ' + msg.toString('hex'));

	leakyBufferFn(msg);

	// todo: grab buffer in...
	//0 -> msg.length

	//var filename = moment().format() + ".jpg";
	//fs.writeFileSync(filename, msg);




//	// echo it back
//	udpSocket.send(msg, 0, msg.length, rinfo.port, rinfo.address, function(err) {
//		if (err) {
//			console.log('error during send ' + err);
//		}
//		else {
//			console.log('echoed');
//		}
//	});

});

udpSocket.bind(port);