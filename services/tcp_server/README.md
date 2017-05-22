Simple Node.js Server for Reciving Photos
---

This program listens on port 5550 for connections from your device running the "tcp_photographer" code in the 
firmware folder.  It uses a lossy rolling time window to separate photos from eachother, 
so it's not the most robust or reliable server code, but it does make it extremely quick to use and the code easier 
to read.  If you want to use this in production, this server would benefit from encryption, 
and some header metadata (message length, type, etc).

How to setup
---

Make sure you have Node.js installed, and install the dependencies in this folder:

```
 cd services/tcp_server
 npm install
 #...
 node main.js
 
 #success!
```

How to run
---

Just run `node main.js`, and as you receive images, it should create an 'images' folder here, 
and start saving photos as "image" + "image number".jpg


Bugs / Issues?
---

Please feel free to open issues on this repo if you find bugs:

https://github.com/dmiddlecamp/particle_arducam/issues

Thanks!
