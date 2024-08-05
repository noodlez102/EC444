const dgram = require('dgram');
const server = dgram.createSocket('udp4');

const PORT = 3333; // Choose a port number

server.on('message', (msg, rinfo) => {
  console.log(`Server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
});

server.on('listening', () => {
  const address = server.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

server.bind(PORT);
