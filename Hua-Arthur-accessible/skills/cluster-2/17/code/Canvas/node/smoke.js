// Modules
var express = require('express');
var app = express(); 
var http = require('http').Server(app);
var io = require('socket.io')(http);
var fs = require('fs');

fs.readFile('smoke.txt', 'utf8', (err, data) => {
  if (err) {
    console.error('Error reading file:', err);
    return;
  }

  const rows = data.trim().split('\n');

  rows.forEach((row, index) => {
    setTimeout(() => {
      const rowData = row.split('\t'); 

      console.log('Parsed Data:', {
        time: parseInt(rowData[0]),
        id: parseInt(rowData[1]),
        smoke: parseInt(rowData[2]),
        temp: parseFloat(rowData[3]),
      });

      const payload = {
        time: parseInt(rowData[0]),
        id: parseInt(rowData[1]),
        smoke: parseInt(rowData[2]),
        temp: parseFloat(rowData[3]),
      };

      io.emit('message', payload);
    }, index * 1000); 
  });
});

app.get('/', function(req, res){
  res.sendFile(__dirname + '/smoke.html');
});

io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

http.listen(3000, function() {
  console.log('listening on localhost:3000');
});