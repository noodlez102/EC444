// Serial port example from design pattern
// Uses ESP code in folder: serial-esp-to-node-serialport
// Modules
var express = require("express");
var app = express(); // Simplified the initialization
var http = require("http").Server(app);
var io = require("socket.io")(http);
const { SerialPort } = require("serialport");
var fs = require("fs");

const { ReadlineParser } = require("@serialport/parser-readline");

// Set the serial port path and baud rate
const port = new SerialPort({
  path: "/dev/cu.usbserial-017481D3",
  baudRate: 115200,
});
const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

// Event handler when the serial port is open
port.on("open", () => {
  console.log("Serial port now open");
});

fs.writeFile("SerialData.txt", "time,Sensor,reading,angle\n", function (err) {
  if (err) throw err;
  console.log("File is created successfully.");
});

// Event handler for received data from the serial port
parser.on("data", (data) => {
  // Log the received data
  // console.log('Data:', data);

  // Append the data to the file 'SerialData.txt'
  fs.appendFile("SerialData.txt", data, function (err) {
    if (err) throw err;
  });

  const rowData = data.split(",");

  // Log the parsed data
  console.log("Parsed Data:", {
    time: parseFloat(rowData[0]),
    sensor: rowData[1],
    reading: parseFloat(rowData[2]),
    angle: parseFloat(rowData[3]),
  });

  const payload = {
    time: parseFloat(rowData[0]),
    sensor: rowData[1],
    reading: parseFloat(rowData[2]),
    angle: parseFloat(rowData[3]),
  };

  io.emit("message", payload);
});

// Serve webpage
app.get("/", function (req, res) {
  res.sendFile(__dirname + "/graph.html");
});

// User socket connection
io.on("connection", function (socket) {
  console.log("a user connected");
  socket.on("disconnect", function () {
    console.log("user disconnected");
  });
});

// Listening on localhost:3000
http.listen(3000, function () {
  console.log("listening on *:3000");
});
