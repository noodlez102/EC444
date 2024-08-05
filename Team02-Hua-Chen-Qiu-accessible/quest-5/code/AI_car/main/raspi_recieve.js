const dgram = require('dgram');
const socketIo = require('socket.io');
const server = dgram.createSocket('udp4');
const http = require('http');
const fs = require('fs');
const path = require('path');

const server2 = http.createServer((req, res) => {
  res.statusCode = 200;
  res.setHeader('Content-Type', 'text/html');
  fs.readFile(path.join(__dirname, 'index.html'), (err, data) => {
    if (err) {
      res.end('Error loading HTML file');
    } else {
      res.end(data);
    }
  });
});
const io = socketIo(server2);

let responseMsg = '0'; // Initial response message

const PORT = 3333; // Choose a port number
const hostname = '0.0.0.0'; // Use '0.0.0.0' to allow connections from any IP address
const port2 = 3000;

server.on('message', (msg, rinfo) => {
  console.log(Server got: ${msg} from ${rinfo.address}:${rinfo.port});
  io.emit('message', msg.toString()); // Emitting the message to all clients
  // Send the current response message back to the client
  server.send(responseMsg, rinfo.port, rinfo.address, (err) => {
    if (err) {
      console.error('Error sending response:', err);
    } else {
      console.log('Response sent to client:', responseMsg);
    }
  });
});

server.on('listening', () => {
  const address = server.address();
  console.log(UDP server listening on ${address.address}:${address.port});
});

// Listen for the 'updateResponseMsg' event from the client
io.on('connection', (socket) => {
  socket.on('updateResponseMsg', (newMsg) => {
    responseMsg = newMsg; // Update the response message
    console.log('Response message updated:', responseMsg);
  });
});

server.bind(PORT);

server2.listen(port2, hostname, () => {
  console.log(Server running at http://${hostname}:${port2}/);
});