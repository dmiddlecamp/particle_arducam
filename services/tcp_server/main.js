var fs = require('fs');
var net = require('net');

var settings = {
    ip: "127.0.0.1",
    port: 5550
};

var moment = require('moment');


var _buffer = [];
var _bufferTimer = null;
var leakyDelay = 1000;

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


var server = net.createServer(function (socket) {
    console.log("Connection received");

    socket.on('readable', function() {
        var data = socket.read();
		leakyBufferFn(data);

		// this slows us down
		//		if (data) {
		//			console.log("received data: " + data.length + " bytes");
		//		}
    });
});

server.listen(settings.port, function () {
	console.log('Listening for TCP packets on port ' + settings.port + ' ...');
});


//udpSocket.on('message', function(msg, rinfo) {
//	if (rinfo.size == 0) {
//		return;
//	}
//	console.log("rinfo was ", rinfo);
//	console.log('received [' + rinfo.address + '] (' + msg.length + ' bytes) ' + msg.toString('hex'));
//	l
//});
//
//udpSocket.bind(port);