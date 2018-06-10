var PORT = 8877;
var HOST = '0.0.0.0';

var dgram = require('dgram');
var server = dgram.createSocket('udp4');

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', function (message, remote) {
    var d = new Date().toISOString();
    var msg = message.toString('utf8');
    // msg = msg.replace(/1: /g,"INFO: ");
    // msg = msg.replace(/2: /g,"ERROR: ");
    console.log(d+" "+remote.address + ':' + remote.port +' - ' + msg);
});

server.bind(PORT, HOST);

