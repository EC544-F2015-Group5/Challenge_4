// Socket Variable
var socket = io.connect('/');

// On page load
$(document).ready(function() {
  $('#start').click(function() {
     socket.emit('Start');
    });
  $('#stop').click(function() {
       socket.emit('Stop');
      });

    });
