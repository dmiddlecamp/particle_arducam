#include "ArduCAM.h"


#define BMPIMAGEOFFSET 66

//#define SD_CS D2

// set pin A2 as the slave select for the ArduCAM shield
const int SPI_CS = A2;

// allow us to use itoa() in this scope
extern char* itoa(int a, char* buffer, unsigned char radix);

const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
      0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
      0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
      0x00, 0x00
};


ArduCAM myCAM(OV5642, SPI_CS);


void setup()
{
  //uint8_t vid,pid;
  uint8_t temp;

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
    Serial.println("Checking for camera...");

    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if(temp != 0x55)
    {
      Serial.println("SPI interface Error!");
      Serial.println("myCam.read_reg said " + String(temp));
      delay(1000);
    }
    else {
      break;
    }
  }

//NOTES
//   while (1) {
//    //Check if the camera module type is OV5642
//    myCAM1.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
//    myCAM1.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
//    if ((vid != 0x56) || (pid != 0x42)) {
//      Serial.println(F("Can't find OV5642 module!"));
//      delay(1000); continue;
//    } else {
//      Serial.println(F("OV5642 detected.")); break;
//    }
//  }
//
//Change to JPEG capture mode and initialize the OV5640 module
//  myCAM1.set_format(JPEG);
//  myCAM1.InitCAM();
//  myCAM1.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
//  myCAM1.clear_fifo_flag();
//  myCAM1.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
//
//  //myCAM1.OV5642_set_JPEG_size(OV5642_640x480); delay(1000);
//  myCAM1.OV5642_set_JPEG_size(OV5642_320x240); delay(1000);
//  delay(1000);
//  myCAM1.clear_fifo_flag();
//  Serial.println("Ready:,1");



  Serial.println("Camera found, initializing...");

  //Change MCU mode
  myCAM.write_reg(ARDUCHIP_MODE, 0x00);
  myCAM.InitCAM();
}

void loop()
{
//  char str[8];
//  unsigned long previous_time = 0;
//  static int k = 0;
//  uint8_t temp;
//
//  myCAM.write_reg(ARDUCHIP_MODE, 0x01);		 	//Switch to CAM
//
//  while(1)
//  {
//    temp = myCAM.read_reg(ARDUCHIP_TRIG);
//
//    if(!(temp & VSYNC_MASK))				//New Frame is coming
//    {
//       myCAM.write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU
////       myGLCD.resetXY();
//       myCAM.write_reg(ARDUCHIP_MODE, 0x01);    	//Switch to CAM
//       while(!(myCAM.read_reg(ARDUCHIP_TRIG)&0x01));	//Wait for VSYNC is gone
//    }
//	else if ((temp & SHUTTER_MASK))
//	{
//       previous_time = millis();
//       while(myCAM.read_reg(ARDUCHIP_TRIG) & SHUTTER_MASK)
//       {
//         if((millis() - previous_time) > 1500)
//         {
//           Playback();
//         }
//       }
//       if((millis() - previous_time) < 1500)
//       {
//         k = k + 1;
//         itoa(k, str, 10);
//         strcat(str,".bmp");				//Generate file name
//         myCAM.write_reg(ARDUCHIP_MODE, 0x00);    	//Switch to MCU, freeze the screen
//         GrabImage(str);
//       }
//    }
//  }
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
//      //Write image data to bufer if not full
//      if(k >= 256)
//      {
//        //Write 256 bytes image data to file from buffer
//        //outFile.write(buf,256);
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

