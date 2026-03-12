const express = require('express');
const path = require('path');
const app = express();
const port = process.env.PORT || 3000;

app.use(express.static(path.join(__dirname, 'public')));
app.use('/modules', express.static(path.join(__dirname, 'node_modules')));

app.get(['/', '/robot', '/simulator'], (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

const server = app.listen(port, '0.0.0.0', () => {
  console.log(`Robot arm simulator running at http://localhost:${port}`);
  console.log(`Also accessible on local network when your machine IP is used`);
  console.log(`Try custom URL: http://localhost:${port}/robot`);
});

server.on('error', (err) => {
  if (err.code === 'EADDRINUSE') {
    console.error(`Port ${port} is already in use. Try a different port: PORT=3001 npm start`);
    process.exit(1);
  } else {
    throw err;
  }
});
