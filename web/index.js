var express = require('express');
var app = express();
var server = app.listen(3000);
var io = require('socket.io')(server);
var port = process.env.PORT || 3000;

app.use('/', express.static('public'));
console.log("Running server on port", port)
var users = 0;

io.on('connection', function (socket) {

	console.log("CONNECTED")

	socket.on("data", function(player, position) {
		console.log("DATA", player, position)
		io.emit("data", player, position);
	})

	socket.on("disconnect", function() {
		console.log(`User disconnected. Currently ${users} users`);
		socket.emit("user count", users);
	});

});

