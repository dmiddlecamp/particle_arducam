# particle_arducam


how to build
---

    ./build.sh sandbox


how to flash
---

    ./flash.sh sandbox


how to see if it's working
---

    particle serial monitor --follow

Hoping to see something like "Camera found, initializing..."


how it's wired
---

This is probably wrong, but it's what I have so far, feedback very welcome.

Photon -- Camera -- Purpose
A2 -- CS -- SS

|Photon	|Camera	|Purpose|
|---	|---	|---	|
|A2   	|CS   	|SS   	|
|A3   	|SCK   	|SCK   	|
|A4   	|MOSI  	|MOSI  	|
|A5   	|MISO  	|MISO  	|

