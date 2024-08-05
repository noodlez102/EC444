const dgram = require("dgram");
const socketIo = require("socket.io");
const server_esp32 = dgram.createSocket("udp4");
const http = require("http");
const fs = require("fs");
const path = require("path");
const net = require("net");

// Difference among server, server2, server3
// server: UDP server: This server listens for incoming UDP packets from the ESP32 and sends responses back to it
// server2: HTTP server: This server serves the HTML page that contains the client-side JavaScript code
// server3: TCP server: This server listens for incoming connections from the Python script and receives data from it

const port_esp32 = 3333; // Choose a port number
const port2_web = 3000;
const port3_python = 3334; // Port number to listen on
let gestureCount = { 1: 0, 2: 0, 3: 0 };
let python_msg = "";
let responseMsg = "0"; // Initial response message
const hostname = "0.0.0.0"; // Use '0.0.0.0' to allow connections from any IP address
let last = "";
const server2_web = http.createServer((req, res) => {
  res.statusCode = 200;
  res.setHeader("Content-Type", "text/html");
  fs.readFile(path.join(__dirname, "index.html"), (err, data) => {
    if (err) {
      res.end("Error loading HTML file");
    } else {
      res.end(data);
    }
  });
});

const io = socketIo(server2_web);

server_esp32.on("message", (msg, rinfo) => {
  console.log(`Server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
  io.emit("message", msg.toString()); // Emitting the message to all clients

  // Send the current response message back to the client
  server_esp32.send(responseMsg, rinfo.port, rinfo.address, (err) => {
    if (err) {
      console.error("Error sending response:", err);
    } else {
      responseMsg = python_msg;
      console.log("Response sent to client:", responseMsg);
    }
  });
});

server_esp32.on("listening", () => {
  const address = server_esp32.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

// Create a TCP server
const server3_python = net.createServer((socket) => {
  console.log("Client connected:", socket.remoteAddress);

  // Listen for data from the client
  socket.on("data", (data) => {
    console.log("Data received from client:", data.toString());
    python_msg = data.toString();
    const gesture = parseInt(data.toString());
    if(last != gesture){
    gestureCount[gesture]++;
    last = gesture;
    }

    io.emit("gestureData", gestureCount);
    console.log(gestureCount)
    // You can process the data received from the Python script here
  });

  // Handle client connection termination
  socket.on("end", () => {
    console.log("Client disconnected");
  });

  // Handle errors
  socket.on("error", (err) => {
    console.error("Socket error:", err);
  });
});

io.on("connection", (socket) => {
  console.log("A user connected");

  // Handle disconnections
  socket.on("disconnect", () => {
    console.log("A user disconnected");
  });
});

server_esp32.bind(port_esp32);
server2_web.listen(port2_web, hostname, () => {
  console.log(`Web Server running at http://${hostname}:${port2_web}/`);
});
// Start listening on the specified port
server3_python.listen(port3_python, () => {
  console.log(`Python Server listening on port ${port3_python}`);
});