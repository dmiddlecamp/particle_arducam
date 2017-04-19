#include "ArduCAM.h"
#include "memorysaver.h"
SYSTEM_THREAD(ENABLED);

#define VERSION_SLUG "7n"

TCPClient client;

#define SERVER_ADDRESS "192.168.2.34"
#define SERVER_TCP_PORT 5550

#define TX_BUFFER_MAX 1024
uint8_t buffer[TX_BUFFER_MAX + 1];
int tx_buffer_index = 0;




//#define SD_CS D2

// set pin A2 as the slave select for the ArduCAM shield
const int SPI_CS = A2;

// allow us to use itoa() in this scope
//extern char* itoa(int a, char* buffer, unsigned char radix);
//#define BMPIMAGEOFFSET 66
//const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
//{
//      0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
//      0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
//      0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
//      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
//      0x00, 0x00
//};


ArduCAM myCAM(OV5642, SPI_CS);


void setup()
{
  Particle.publish("status", "Good morning, Version: " + String(VERSION_SLUG));
  delay(1000);

  uint8_t vid,pid;
  uint8_t temp;

  Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.begin();

  Serial.begin(115200);
  Serial.println("ArduCAM Start!");

  // set the SPI_CS as an output:
  pinMode(SPI_CS, OUTPUT);

  // initialize SPI:
  SPI.begin();
  //SPI.begin(SPI_MODE_MASTER);
  //SPI.begin(SPI_MODE_SLAVE, SPI_CS);


  while(1) {
    Particle.publish("status", "checking for camera");
    Serial.println("Checking for camera...");

    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if(temp != 0x55)
    {
      Serial.println("SPI interface Error!");
      Serial.println("myCam.read_reg said " + String(temp));
      delay(5000);
    }
    else {
      break;
    }
    Particle.process();
  }

    Particle.publish("status", "Camera found.");


while(1){
  //Check if the camera module type is OV5642
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)){
    Serial.println(F("Can't find OV5642 module!"));
    Particle.publish("status", "Not found, camera says " + String::format("%d:%d", vid, pid));
    delay(5000);
    continue;
  }
  else {
    Serial.println(F("OV5642 detected."));
    Particle.publish("status", "OV5642 detected: " + String::format("%d:%d", vid, pid));
    break;
  }
}


  Serial.println("Camera found, initializing...");
    //myCAM.write_reg(ARDUCHIP_MODE, 0x01);		 	//Switch to CAM

    //Change MCU mode
    myCAM.set_format(JPEG);
    delay(100);

    myCAM.InitCAM();
    delay(100);

//    myCAM.set_format(JPEG);
//    delay(100);

    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    //myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    delay(100);

    myCAM.clear_fifo_flag();
    delay(100);

    myCAM.write_reg(ARDUCHIP_FRAMES,0x00);
    delay(100);

    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    //myCAM.OV5640_set_JPEG_size(OV5642_1600x1200);
    //myCAM.OV5640_set_JPEG_size(OV5642_640x480);    // ?
    //delay(100);

    // wait a sec`
    delay(1000);


    client.connect(SERVER_ADDRESS, SERVER_TCP_PORT);
}



void loop()
{
    if (!client.connected()) {
        //client.stop();
        Particle.publish("status", "Attempting to reconnect to TCP Server...");
        if (!client.connect(SERVER_ADDRESS, SERVER_TCP_PORT)) {
            delay(1000);
            return;
        }
    }

    Particle.publish("status", "Taking a picture...");
    Serial.println("Taking a picture...");


    //myCAM.OV5640_set_JPEG_size(OV5640_320x240);   //works
    //myCAM.OV5640_set_JPEG_size(OV5642_1600x1200); //doesn't work
    //myCAM.OV5640_set_JPEG_size(OV5642_1280x960);  // doesn't work?
    //myCAM.OV5640_set_JPEG_size(OV5642_640x480);    // ?

    myCAM.OV5642_set_JPEG_size(OV5642_2592x1944); //works
    delay(100);

    //myCAM.set_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);
    //myCAM.clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK);


    myCAM.flush_fifo();
    delay(100);

    myCAM.clear_fifo_flag();
    delay(100);

    myCAM.start_capture();
    delay(100);



    //myCAM.start_capture();
    unsigned long start_time = millis(),
                  last_publish = millis();


//
//  wait for the photo to be done
//
    while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)) {
        Particle.process();
        delay(10);

        unsigned long now = millis();
        if ((now - last_publish) > 1000) {
            Particle.publish("status", "waiting for photo " + String(now-start_time));
            last_publish = now;
        }

        if ((now-start_time) > 30000) {
            Particle.publish("status", "bailing...");
            break;
        }
    }
    delay(100);

    int length = myCAM.read_fifo_length();
    Particle.publish("status", "Image size is " + String(length));
    Serial.println("Image size is " + String(length));

    uint8_t temp = 0xff, temp_last = 0;
    int bytesRead = 0;



    if(myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
        delay(100);
        Serial.println(F("ACK CMD CAM Capture Done."));
        Particle.publish("status", "Capture done");

        //myCAM.CS_LOW();

        myCAM.set_fifo_burst();


        //#if !(defined (OV5642_MINI_5MP_PLUS) ||(defined (ARDUCAM_SHIELD_V2) && defined (OV5642_CAM)))
        //SPI.transfer(0xFF);
        //#endif

        tx_buffer_index = 0;
        temp = 0;

        //while (bytesRead < length)
        while( (temp != 0xD9) | (temp_last != 0xFF) )
        {
            temp_last = temp;
            temp = myCAM.read_fifo();
            bytesRead++;


            buffer[tx_buffer_index++] = temp;

            if (tx_buffer_index >= TX_BUFFER_MAX) {
                client.write(buffer, tx_buffer_index);

                tx_buffer_index = 0;
                Particle.process();
            }

            if (bytesRead > 2048000) {
                // failsafe
                break;
            }
        }


        if (tx_buffer_index != 0) {
            client.write(buffer, tx_buffer_index);
        }

        //Clear the capture done flag
        //myCAM.CS_HIGH();
        myCAM.clear_fifo_flag();

        Serial.println(F("End of Photo"));
    }

    Serial.println("sleeping 10 seconds");
    Particle.publish("status", "Sleeping 10 seconds");
    delay(10 * 1000);
}
