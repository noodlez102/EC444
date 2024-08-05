const { SerialPort } = require('serialport');
const fs = require('fs');
const port = new SerialPort({ path: 'COM3', baudRate: 115200 });
const { ReadlineParser } = require('@serialport/parser-readline');

const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));

// Specify the file path where you want to save the sensor data
const filePath = 'sensor_data.csv';

// Read the port data
port.on("open", () => {
  console.log('Serial port now open');
});

parser.on('data', data => {
  console.log('The word from ESP32:', data);
  const currentTime = new Date().toISOString();
  const sensorData = `${currentTime},${data}\n`; // Format: current time, sensor value
  // Append the sensor data to the CSV file
  fs.writeFile(filePath, sensorData, { flag: 'a+' }, err => {
    if (err) {
      console.error('Error writing to file:', err);
    } else {
      console.log('Sensor data written to file successfully.');
    }
  });
});
