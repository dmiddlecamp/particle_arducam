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


what camera / sensor I'm using
---

    https://www.amazon.com/Arducam-Module-Camera-Arduino-Mega2560/dp/B013JUKZ48


how it's wired
---

|Photon	|Camera	|Purpose|
|---	|---	|---	|
|A2   	|CS   	|SS   	|
|A3   	|SCK   	|SCK   	|
|A4   	|MISO  	|MISO  	|
|A5   	|MOSI  	|MOSI  	|
|D0   	|SDA    |SDA    |
|D1   	|SCL    |SCL    |

