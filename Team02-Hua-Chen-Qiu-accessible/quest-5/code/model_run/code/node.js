const dgram = require('dgram');
const socketIo = require('socket.io');
const server = dgram.createSocket('udp4');
const http = require('http');
const fs = require('fs');
const path = require('path');
const net = require('net');

const port3 = 3334; // Change to the same port number used in the Python script
let server3Data = ''; // Variable to store data received by server3

const server3 = net.createServer((socket) => {
    console.log('Client connected');

    // Handle incoming data
    socket.on('data', (data) => {
        server3Data = data.toString(); // Store the data received from server3
        console.log('Received data from server3:', server3Data);
        // Process the data as needed
    });

    // Handle connection termination
    socket.on('end', () => {
        console.log('Client disconnected');
    });
});

server3.on('error', (err) => {
    console.error('Server error:', err);
});

server3.listen(port3, () => {
    console.log(`Server listening on port ${port3}`);
});

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
const csvFile = 'test_senso/frame.csv'; // CSV file path

server.on('message', (msg, rinfo) => {
    console.log(`Server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
    io.emit('message', msg.toString()); // Emitting the message to all clients

    // Save the message to the first line of the CSV file
    const timestamp = new Date().toISOString();
    const messageData = `${msg},${responseMsg}\n`; 
    //messageData = messageData;// Use server3Data instead of responseMsg
    fs.writeFile(csvFile, messageData, (err) => {
        if (err) {
            console.error('Error saving message to CSV:', err);
        } else {
            console.log('Message saved to CSV file');
        }
    });

    // Send the data received from server3 back to the client
    server.send(server3Data, rinfo.port, rinfo.address, (err) => {
        if (err) {
            console.error('Error sending response:', err);
        } else {
            console.log('Data from server3 sent to client:', server3Data);
        }
    });
});

server.on('listening', () => {
    const address = server.address();
    console.log(`UDP server listening on ${address.address}:${address.port}`);
});

// Listen for the 'updateResponseMsg' event from the client
io.on('connection', (socket) => {
    socket.on('updateResponseMsg', (newData) => {
        // Append the new data to the front of the response message
        responseMsg = newData;
        console.log('Response message updated:', responseMsg);
    });
});

server.bind(PORT);
server2.listen(port2, hostname, () => {
    console.log(`Server running at http://${hostname}:${port2}/`);
});
