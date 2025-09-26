// server.js
import express from "express";
import { WebSocketServer } from "ws";
import http from "http";

const app = express();
const server = http.createServer(app);

// Create WebSocket server on top of HTTP server
const wss = new WebSocketServer({ server });

wss.on("connection", (ws) => {
  console.log("Client connected to WebSocket");

  ws.on("close", () => {
    console.log("Client disconnected");
  });


  ws.on("close", () => clearInterval(interval));
});

app.post("/api/sms", (req, res) => {
  const smsData = req.body;
  console.log("✅ Received data from phone shortcut:", smsData);

  // Broadcast the received data to all connected WebSocket clients
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(smsData));
    }
  });

  // Send a success response back to the iPhone's shortcut
  res.status(200).json({ status: "Data received and broadcasted" });
});




// Start server
const PORT = 4000;
server.listen(PORT, () => {
  console.log(`Backend running on http://localhost:${PORT}`);
});
