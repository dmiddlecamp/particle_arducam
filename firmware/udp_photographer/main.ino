#include "ArduCAM.h"
#include "memorysaver.h"
SYSTEM_THREAD(ENABLED);


UDP Udp;
#define UDP_BROADCAST_PORT 5550
IPAddress broadcastAddress(255,255,255,255);

//#define SERVER_ADDRESS "192.168.2.34"

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
  Particle.publish("status", "Good morning");
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

    myCAM.set_format(JPEG);
    delay(100);

    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    //myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    delay(100);

    myCAM.clear_fifo_flag();
    delay(100);

    myCAM.write_reg(ARDUCHIP_FRAMES,0x00);
    delay(100);

    //myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    //myCAM.OV5640_set_JPEG_size(OV5642_1600x1200);
    myCAM.OV5640_set_JPEG_size(OV5642_640x480);    // ?

    delay(100);

    // wait a sec`
    delay(1000);

    Udp.setBuffer(1024);
    Udp.begin(UDP_BROADCAST_PORT);
}



void loop()
{
    Particle.publish("status", "Taking a picture... 7l");
    Serial.println("Taking a picture...");


    //myCAM.OV5640_set_JPEG_size(OV5640_320x240);   //works
    //myCAM.OV5640_set_JPEG_size(OV5642_1600x1200); //doesn't work
    //myCAM.OV5640_set_JPEG_size(OV5642_1280x960);  // doesn't work?

    myCAM.OV5640_set_JPEG_size(OV5642_640x480);    // ?

    //myCAM.OV5642_set_JPEG_size(OV5642_2592x1944); //works
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

//    if (length > 128000) {
//        //santiy check for the moment..
//        length = 128000;
//    }

    //String preamble = "IMAGE:" + String(length) + ":";
   //Udp.sendPacket(preamble.c_str(), preamble.length(), broadcastAddress, UDP_BROADCAST_PORT);

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


//        // a better way to read off the fifo buffer?
//          static const size_t bufferSize = 4096;
//          static uint8_t buffer[bufferSize] = {0xFF};
//          while (length) {
//              size_t will_copy = (length < bufferSize) ? length : bufferSize;
//              myCAM.transferBytes(&buffer[0], &buffer[0], will_copy);
//              //if (!client.connected()) break;
//              //client.write(&buffer[0], will_copy);
//              length -= will_copy;
//          }


        tx_buffer_index = 0;
        temp = 0;
        Serial.println(F("ACK IMG"));
        //while( (temp != 0xD9) | (temp_last != 0xFF) )

        //while (bytesRead < length)
        while( (temp != 0xD9) | (temp_last != 0xFF) )
        {
          temp_last = temp;

          noInterrupts(); // disable interrupts
          SINGLE_THREADED_BLOCK() {
              delayMicroseconds(15);
              temp = myCAM.read_fifo();
          }
          interrupts();   // enable interrupts
          bytesRead++;


          buffer[tx_buffer_index++] = temp;

          if (tx_buffer_index >= TX_BUFFER_MAX) {

//            for(int i=0;i<tx_buffer_index;i++) {
//                Serial.print(buffer[i], HEX);
//            }

            Udp.sendPacket(buffer, tx_buffer_index, broadcastAddress, UDP_BROADCAST_PORT);

            tx_buffer_index = 0;
            Udp.flush();
            delay(20);
            Particle.process();
          }

//          if (bytesRead > length) {
//            break;
//          }

            if (bytesRead > 1024000) {
                // failsafe
                break;
            }
        }


        if (tx_buffer_index != 0) {
            Udp.sendPacket(buffer, tx_buffer_index, broadcastAddress, UDP_BROADCAST_PORT);
        }

        //String photoEnd = ":ENDENDEND";
       // Udp.sendPacket(photoEnd.c_str(), photoEnd.length(), broadcastAddress, UDP_BROADCAST_PORT);

        //Clear the capture done flag
        //myCAM.CS_HIGH();
        myCAM.clear_fifo_flag();

        Serial.println(F("End of Photo"));
    }

    Serial.println("sleeping 10 seconds");
    Particle.publish("status", "Sleeping 10 seconds");
    delay(10 * 1000);


//  char str[8];
//  unsigned long previous_time = 0;
//  static int k = 0;
//  uint8_t temp;
//
////  myCAM.write_reg(ARDUCHIP_MODE, 0x01);		 	//Switch to CAM
//
//
//    temp = myCAM.read_reg(ARDUCHIP_TRIG);
//
//    if(!(temp & VSYNC_MASK))				//New Frame is coming
//    {
//        myCAM.write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU
//        myCAM.write_reg(ARDUCHIP_MODE, 0x01);    	//Switch to CAM
//        while(!(myCAM.read_reg(ARDUCHIP_TRIG)&0x01));	//Wait for VSYNC is gone
//    }
//    else if ((temp & SHUTTER_MASK))
//    {
//        previous_time = millis();
//        while(myCAM.read_reg(ARDUCHIP_TRIG) & SHUTTER_MASK)
//        {
//            if((millis() - previous_time) > 1500)
//            {
//               Playback();
//            }
//        }
//        if((millis() - previous_time) < 1500)
//        {
//            k = k + 1;
//            itoa(k, str, 10);
//            strcat(str,".bmp");				//Generate file name
//            myCAM.write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU, freeze the screen
//            GrabImage(str);
//        }
//    }


}

//
//
//void GrabImage(char* str)
//{
//
////  File outFile;
//  char VH,VL;
//  uint8_t temp;
//  byte buf[256];
//  static int k = 0;
//  int i,j = 0;
//
////  outFile = SD.open(str,O_WRITE | O_CREAT | O_TRUNC);
////  if (! outFile)
////  {
////    Serial.println("Open File Error");
////    return;
////  }
//
//  //Switch to FIFO Mode
//  myCAM.write_reg(ARDUCHIP_TIM, MODE_MASK);
//  //Flush the FIFO
//  myCAM.flush_fifo();
//  //Start capture
//  myCAM.start_capture();
//  Serial.println("Start Capture");
//
//  //Polling the capture done flag
//  while(!(myCAM.read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK));
//  Serial.println("Capture Done!");
//
//  k = 0;
//  //Write the BMP header
//  for( i = 0; i < BMPIMAGEOFFSET; i++)
//  {
//    char ch = pgm_read_byte(&bmp_header[i]);
//    buf[k++] = ch;
//  }
//  //outFile.write(buf,k);
//
//  //------------------------------
//  //---Serial print here    outFile.write(buf,k);
//  //------------------------------
//
//  //Read first dummy byte
//  //myCAM.read_fifo();
//
//  k = 0;
//  //Read 320x240x2 byte from FIFO
//  //Save as RGB565 bmp format
//  for(i = 0; i < 240; i++)
//    for(j = 0; j < 320; j++)
//  {
//      VH = myCAM.read_fifo();
//      VL = myCAM.read_fifo();
//      buf[k++] = VL;
//      buf[k++] = VH;
//
//      Serial.print(String::format("y:%d x:%d VH:%d VL:%d", i, j, VL, VH));
//
//      //Write image data to bufer if not full
//      if(k >= 256)
//      {
//        //Write 256 bytes image data to file from buffer
//        //outFile.write(buf,256);
//
//        //Serial.println("data: " + String(buf, HEX));
//          //------------------------------
//          //---Serial print here    outFile.write(buf,256);
//          //------------------------------
//
//        k = 0;
//      }
//  }
//  //Close the file
//  //outFile.close();
//  //Clear the capture done flag
//  myCAM.clear_fifo_flag();
//
//  //Switch to LCD Mode
//  myCAM.write_reg(ARDUCHIP_TIM, 0);
//  return;
//}
//
////void GrabImage(char* str)
////{
////
////  File outFile;
////  char VH,VL;
////  uint8_t temp;
////  byte buf[256];
////  static int k = 0;
////  int i,j = 0;
////
////  outFile = SD.open(str,O_WRITE | O_CREAT | O_TRUNC);
////  if (! outFile)
////  {
////    Serial.println("Open File Error");
////    return;
////  }
////
////  //Switch to FIFO Mode
////  myCAM.write_reg(ARDUCHIP_TIM, MODE_MASK);
////  //Flush the FIFO
////  myCAM.flush_fifo();
////  //Start capture
////  myCAM.start_capture();
////  Serial.println("Start Capture");
////
////  //Polling the capture done flag
////  while(!(myCAM.read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK));
////  Serial.println("Capture Done!");
////
////  k = 0;
////  //Write the BMP header
////  for( i = 0; i < BMPIMAGEOFFSET; i++)
////  {
////    char ch = pgm_read_byte(&bmp_header[i]);
////    buf[k++] = ch;
////  }
////  outFile.write(buf,k);
////  //Read first dummy byte
////  //myCAM.read_fifo();
////
////  k = 0;
////  //Read 320x240x2 byte from FIFO
////  //Save as RGB565 bmp format
////  for(i = 0; i < 240; i++)
////    for(j = 0; j < 320; j++)
////  {
////      VH = myCAM.read_fifo();
////      VL = myCAM.read_fifo();
////      buf[k++] = VL;
////      buf[k++] = VH;
////      //Write image data to bufer if not full
////      if(k >= 256)
////      {
////        //Write 256 bytes image data to file from buffer
////        outFile.write(buf,256);
////        k = 0;
////      }
////  }
////  //Close the file
////  outFile.close();
////  //Clear the capture done flag
////  myCAM.clear_fifo_flag();
////
////  //Switch to LCD Mode
////  myCAM.write_reg(ARDUCHIP_TIM, 0);
////  return;
////}
//
//
//void Playback()
//{
///*
//  File inFile;
//  char str[8];
//  int k = 0;
//  myCAM.write_reg(ARDUCHIP_MODE, 0x00);    		//Switch to MCU
////  myGLCD.InitLCD(PORTRAIT);
//
//  while(1)
//  {
//      k = k + 1;
//      itoa(k, str, 10);
//      strcat(str,".bmp");
//      inFile = SD.open(str,FILE_READ);
//      if (! inFile)
//        return;
//      myGLCD.clrScr();
//      //myGLCD.resetXY();
//      myGLCD.dispBitmap(inFile);
//      inFile.close();
//      delay(2000);
//  }
//*/
//}
//
//
