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

  // send mock data every 2 sec
  const interval = setInterval(() => {
    const data = {
      trainId: "TRN-4521",
      imu: {
        x: (Math.random() * 10).toFixed(2),
        y: (Math.random() * 10).toFixed(2),
        z: (Math.random() * 10).toFixed(2),
      },
      trackStiffness: (Math.random() * 100).toFixed(2),
      timestamp: new Date().toISOString(),
    };
    ws.send(JSON.stringify(data));
  }, 2000);

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
