#define A2DP
#define SDCARD

#define WIFI

#ifdef WIFI
  #define WEBSERVER
  //#define WIFIUDP
#endif


//#include <EEPROM.h>
//#include "esp_bt_main.h"
//#include "esp_bt_device.h"
//#include "esp_vfs_fat.h"
//#include "sdmmc_cmd.h"
//#include "analogWrite.h"

#include "wifidef.h"
#include "wav_head.h"
#include <Wire.h>

#ifdef WIFI
#include <WiFi.h>
#include <ESPmDNS.h>
#endif

#include <driver/i2s.h>
#include <TFT_eSPI.h> // Hardware-specific library

#ifdef SDCARD
#include <SD.h>
#include <FS.h>
#endif

#include <SPI.h>
#ifdef A2DP
#include "BluetoothA2DPSource.h"
#endif
#ifdef WIFIUDP
#include "AsyncUDP.h"
#endif
#ifdef WEBSERVER
#include <WebServer.h>
#endif


#include "Bounce.h"


static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"


#define BUTTON2 35 
#define BUTTON1 0


int16_t map24to16(int32_t x);
char * btoa8(int8_t n);
char * btoa8m(int8_t n);
char * btoa8m2(int8_t n);
char * btoa8m3(int8_t n);
char * btoa8m4(int8_t n);
char * btoa16(int16_t n);
char * btoa16m(int16_t n);
char * btoa32(int32_t n);
char * btoa32m(int32_t n);
char * btoa24(int32_t n);
char * btoa24m(int32_t n);
void wavHeader(byte* header, int wavSize);
void dumpWaveHeader(byte *h);
void tft_draw(void);
void i2s_init_mic();
void startRecord(void);
void stopRecord(void);
void displayOff(void);
void displayOn(void);
void initA2DP(void);
void deinitA2DP(void);
void initWiFi(void);
void deinitWiFi(void);



/*
3C:61:05:0B:BA:66
*/

#ifndef ESP_NOW_MAX_DATA_LEN
//#define ESP_NOW_MAX_DATA_LEN 250
//#define ESP_NOW_MAX_DATA_LEN 255
#define ESP_NOW_MAX_DATA_LEN 240
//#define ESP_NOW_MAX_DATA_LEN 480
//#define ESP_NOW_MAX_DATA_LEN 960
//#define ESP_NOW_MAX_DATA_LEN 16
#endif

//#define MAP_SCALE_DEFAULT 250
#define MAP_SCALE_DEFAULT 80
#define MAP_SCALE8_DEFAULT 80
#define MAP_SCALE16_DEFAULT 160
#define MAP_SCALE32_DEFAULT 50000

#define MIC_I2S_WS 15
#define MIC_I2S_SD 13
#define MIC_I2S_SCK 2

#define SD_I2S_MISO GPIO_NUM_21
#define SD_I2S_MOSI GPIO_NUM_22
#define SD_I2S_SCK GPIO_NUM_27
#define SD_I2S_CS GPIO_NUM_33

#define I2S_PORT I2S_NUM_0

#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#define SPI_DMA_CHAN    host.slot
#else
#define SPI_DMA_CHAN    1
#endif

#define HEADER_SIZE 44
//#define WAV_SAMPLE_RATE 16000
//#define WAV_SAMPLE_RATE 32000
#define WAV_SAMPLE_RATE 44100
//#define WAV_SAMPLE_RATE 22050
#define WAV_NUM_CHAN 1
//#define WAV_BIT_PER_SAMPLE 16
//#define WAV_BIT_PER_SAMPLE 24
#define WAV_BIT_PER_SAMPLE 32
//#define WAV_BYTE_PER_SAMPLE 2 //  
//#define WAV_BYTE_PER_SAMPLE 3
#define WAV_BYTE_PER_SAMPLE 4

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN           TFT_BL 


// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

//#define WAV_BIT_PER_SAMPLE 24
//#define WAV_BYTE_RATE 2
//#define WAV_BYTE_RATE 4
//.sample_rate = 8000, 
//.sample_rate = 11025
//.sample_rate = 16000
//.sample_rate = 22500
//.sample_rate = 32000
//.sample_rate = 44100


extern const char *headByteLabels[44];



TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite spr = TFT_eSprite(&tft);
#ifdef A2DP
BluetoothA2DPSource a2dp_source;
#endif

#ifdef SDCARD
SPIClass * xspi = NULL;
#endif



#ifdef WEBSERVER
WebServer server(80);
#endif

int samplesRead = 0;
size_t bytesRead = 0;

int16_t rawBuffer16[ESP_NOW_MAX_DATA_LEN] = {0};
//uint8_t rawBuffer16[ESP_NOW_MAX_DATA_LEN*2] = {0};
uint8_t rawBuffer24[ESP_NOW_MAX_DATA_LEN * 3] = {0};
int32_t rawBuffer32[ESP_NOW_MAX_DATA_LEN] = {0};
uint8_t buffer8[ESP_NOW_MAX_DATA_LEN * 4] = {0};
int32_t buffer32[ESP_NOW_MAX_DATA_LEN] = {0};
char buffer[180] = {0};
char outBuffer[480];

unsigned long ms = 0;
byte header[HEADER_SIZE];
byte header2[HEADER_SIZE];
File recFile;
uint32_t totalRecBytes = 0;

int32_t max32_y = MAP_SCALE32_DEFAULT;
int32_t map32Scale = MAP_SCALE32_DEFAULT;
unsigned long lastScale = 0;
unsigned long lastBluetoothConnect = 0;
unsigned long lastButton2Hit = 0;
unsigned long button2HitsInLast2Seconds = 0;
Bounce bbutton1 = Bounce(BUTTON1,15); 
Bounce bbutton2 = Bounce(BUTTON2,15); 
TaskHandle_t task_tft_draw;


#ifdef A2DP
/* I2S_DAC_CHANNEL_BOTH_EN */
OneChannelSoundData monoOut;
#endif


bool a2dp_on = false;
bool a2dp_started = false;

bool record = false;
bool started_record = false;

bool debug_output = false;

bool display_on = true;
bool menu_on = false;
bool draw_menu = true;
bool backlight_on = true;
bool read_mic_on = true;

bool backlight_dim = false;
bool soft_ap = false;

char ip[17] = {0};


// ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU *****

typedef void (*MenuEvent)(int,byte);

char* mainMenu[] = {
  "Record",
  "WiFi",
  "BlueTooth",
  "Brightness",
  "DisplayOff",
  "Back",


};

char **currentMenu = mainMenu;
int currentMenuSelection = 0;
uint8_t currentMenuSize = sizeof(mainMenu)/sizeof(char *);

void nullEvent( int control, byte ival ) {
  Serial.println("NullEvent");
}

void mainMenuEvent( int control, byte ival ) {

  static uint8_t dim_amount = 255;

  if( currentMenuSelection == currentMenuSize-1 ) {
    menu_on = false;
    draw_menu = false;
    return;
  }


  switch(currentMenuSelection) {
    case 0: //record
      Serial.println("Record");
      !record?startRecord():stopRecord();

    break;
    case 1: //WiFi 
      Serial.println("WiFi");
      initWiFi();
    break;
    case 2: //bluetooth 
      Serial.println("BT");
      initA2DP();
    break;
    case 3: //brightness 
      ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
      ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
      ledcAnalogWrite(LEDC_CHANNEL_0, dim_amount);
    break;
    case 4: //disp off 
      displaySleep();
    break;

  } 

}

//MenuEvent currentMenuEvent = &nullEvent;
MenuEvent currentMenuEvent = &mainMenuEvent;



void displayMenu(void) {

  uint8_t margin = 10;
  uint8_t y = 0; 
  if( draw_menu ) { 

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(margin, margin, 1);
    tft.setTextSize(1);
    

    for( int x = 0; x < currentMenuSize; x++) {
      if( x == currentMenuSelection ) {
        tft.setTextColor(TFT_BLACK,TFT_WHITE);  
        tft.print(currentMenu[x]);
      } else {
        tft.setTextColor(TFT_WHITE,TFT_BLACK);  
        tft.print(currentMenu[x]);
      }
      //y += 25; //TextSize 2
      y += 10; //TextSize 2
      tft.setCursor(margin, margin+y, 1);

    }

    tft.setTextSize(2);
    draw_menu = false;

  }

}

// ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU ***** ***** MENU *****


















// ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP *****
#ifdef A2DP


void deinitA2DP(void) {

  Serial.println("Disabling Bluetooth");
  //ESP.reset();
  //a2dp_started = false;

}



void initA2DP(void) {

  deinitWiFi();

  Serial.println("Starting Bluetooth");
  a2dp_source.start("BSR36");
  Serial.println("Completed Setup");

  if( a2dp_source.isConnected() && !a2dp_started ) {

    a2dp_started = true;
    tft.setCursor(0, 0, 1);
    tft.print("Connected  BSR36!!!");
    Serial.println("Connected BSR36!!!");
    delay(1500);
  } 
  
  if( !a2dp_source.isConnected() && a2dp_started ) {
    a2dp_started = false;
    tft.setCursor(0, 0, 1);
    tft.print("Disconnected  BSR36!!!");
    Serial.println("Disconnected  BSR36!!!");
    delay(1500);
  }

  if( a2dp_started ) {
    //monoOut.setData(rawBuffer16,samplesRead);
    monoOut.setData(rawBuffer16,samplesRead);
    //SoundData *data = new OneChannelSoundData(rawBuffer16,120);
    if( !a2dp_source.writeData(&monoOut) ) Serial.println("a2dp writeData failed");
  }

 
}
#endif
// ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP ***** ***** A2DP *****







// ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI *****
void deinitWiFi(void) {

  if( soft_ap ) { 
    WiFi.softAPdisconnect(true);
  } else {
    WiFi.disconnect(true);
  }

}

void initWiFi(void) {
  static unsigned long conn_start = 0;
  static bool home_conn = false;

  const char * ssid = WIFI_SSID;
  const char * pwd = WIFI_PWD;

  const char * apssid = WIFI_AP_SSID; 
  const char * appwd = WIFI_AP_PWD;

  IPAddress local_ip(192,168,1,1);
  IPAddress gateway(192,168,1,1);
  IPAddress subnet(255,255,255,0);

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.print("Home WiFi? ");
  tft.setCursor(0, 50, 1);




  conn_start = millis();

  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.print(".");
    if( millis() - conn_start  > 10000 ) break;
  }
  
  if( WiFi.status() != WL_CONNECTED ) {

      soft_ap = true;

      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0, 0, 1);
    

      Serial.print("Setting soft-AP configuration ... ");
      Serial.println(WiFi.softAPConfig(local_ip, gateway, subnet) ? "Ready" : "Failed!");

      Serial.print("Setting soft-AP ... ");
      Serial.println(WiFi.softAP(apssid, appwd) ? "Ready" : "Failed!");

      Serial.print("Soft-AP IP address = ");
      Serial.println(WiFi.softAPIP());

      IPAddress apip = WiFi.softAPIP();
      Serial.println(buffer);

      if( MDNS.begin("mic") ) {
        MDNS.addService("http","tcp",80);
      }

      sprintf(buffer,"SSID: %s\nPW: %s\nIP: %s\nhttp://mic.local",apssid, appwd,apip.toString().c_str());
      tft.setCursor(0, 0, 1);
      tft.print(buffer);
      sprintf(ip,"%s", apip.toString().c_str());
      delay(1000);

 
  } else {
      Serial.println();
      Serial.println("WiFi connected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      IPAddress locip = WiFi.localIP();

      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0, 0, 1);
      sprintf(buffer,"WiFi IP: %s",locip.toString().c_str());
      sprintf(ip,"%s",locip.toString().c_str());
      tft.print(buffer);
      delay(3000);
  }

  #ifdef WEBSERVER
    Serial.println("Initializing WebServer");
    server.on("/",HTTP_GET,onGetRoot);
    server.on("/draw",HTTP_GET,onGetToggleDraw);
    server.on("/bl",HTTP_GET,onGetToggleBacklight);
    server.on("/mic",HTTP_GET,onGetToggleReadMic);
    server.on("/dim",HTTP_GET,onGetDimScreen);
    server.on("/rec.wav",HTTP_GET,onGetWav);
    server.begin();
    Serial.println("Initialized WebServer");
  #endif



}
// ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI ***** ***** WIFI *****

















/* ***** ROUTES ON GETZ/POSTZ N SHIT *******  ***** ROUTES ON GETZ/POSTZ N SHIT *******  ***** ROUTES ON GETZ/POSTZ N SHIT *******/ 
#ifdef WEBSERVER
void onGetRoot() {
  Serial.println("GET /");
  server.send(200, "text/html", "<html><head></head><body>Helloooooo</body></html>");
}

void onGetToggleDraw() {
  display_on = display_on?false:true;
  sprintf(buffer,"GET /draw %s",display_on?"on":"off");
  Serial.println(buffer);
  server.send(200, "text/plain", buffer);
}

void onGetToggleBacklight() {
  ledcDetachPin(TFT_BL);
  backlight_on = backlight_on?false:true;
  backlight_on?digitalWrite(TFT_BL,1):digitalWrite(TFT_BL,0);
  sprintf(buffer,"GET /bl %s",backlight_on?"on":"off");
  Serial.println(buffer);
  server.send(200, "text/plain", buffer);
}

void onGetToggleReadMic() {
  read_mic_on = read_mic_on?false:true;
  sprintf(buffer,"GET /mic %s",read_mic_on?"on":"off");
  Serial.println(buffer);
  server.send(200, "text/plain", buffer);
}


void onGetDebug() {

  tft.writecommand(ST7789_SLPIN);
  tft.writecommand(ST7789_DISPOFF);    //Display off
  ledcDetachPin(TFT_BL);
  digitalWrite(TFT_BL,0);
  display_on = false;
  backlight_on = false;
  read_mic_on= true;

  debug_output = debug_output?false:true;
  sprintf(buffer,"GET /debug %s",debug_output?"on":"off");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.print("download started");
  server.send(200, "text/plain", buffer);
}

void onGetDimScreen() {

  uint8_t dim_amount = 8;

  if( server.hasArg("v") ) {
    String v = server.arg("v");
    dim_amount = v.toInt();    
  }

  // ********************** BRIGHTNESS LCD TFT_BL *************************
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcAnalogWrite(LEDC_CHANNEL_0, dim_amount);
}


void onGetWav() {

  const char *contentType = "application/octet-stream";
  //contentType = "audio/wav";

  File dataFile = SD.open("/rec.wav");
  /*
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.print("download started");
  */

  if( server.streamFile(dataFile,contentType) != dataFile.size() ) {

      Serial.println("download fail");
      //tft.fillScreen(TFT_BLACK);

      /*
      tft.setCursor(0, 0, 1);
      tft.print("\nfile download fail");
      */
      //delay(2000);

  } 
  dataFile.close();

}
#endif // IF WEBSERVER
/* ***** ROUTES ON GETZ/POSTZ N SHIT *******  ***** ROUTES ON GETZ/POSTZ N SHIT *******  ***** ROUTES ON GETZ/POSTZ N SHIT *******/ 


















// ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP *****
void setup()
{

  Serial.begin(115200);



  Serial.println("Setup TFT...");



  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE,TFT_BLACK);  
  tft.setTextSize(2);
  tft.setCursor(0, 0, 1);
  tft.print("Initializing i2s");

  Serial.println("TFT Initialized");

  delay(1000);
  #ifdef WIFI
    //initWiFi();
  #endif

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);


  i2s_init_mic();
  i2s_start(I2S_PORT);

  Serial.println("i2s Initialized");
  delay(500);

  tft.fillScreen(TFT_BLACK);
  spr.setColorDepth(16);
  //spr.createSprite(TFT_WIDTH,TFT_HEIGHT);
  spr.createSprite(TFT_HEIGHT,TFT_WIDTH);

  //a2dp_source.start("BSR36",get_data_channels);
 
#ifdef SDCARD
  Serial.println("Using XSPI");
  xspi = new SPIClass();
  xspi->begin(SD_I2S_SCK, SD_I2S_MISO, SD_I2S_MOSI, SD_I2S_CS); 
  if (!SD.begin(SD_I2S_CS,*xspi)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
#endif 

  listDir(SD,"/",1);
  //delay(2000);



  /*
  uint32_t hexTest = 0x00003E80;
  

  byte hexByte[4] = {0};

  Serial.print("16000 = ");
  Serial.println(hexTest);
  Serial.print("00003E80 = ");
  Serial.println(hexTest,HEX);
 
  memcpy(&hexByte,&hexTest,4);
  Serial.print("hexByte: ");
  Serial.println(hexByte[0],HEX); 
  Serial.println(hexByte[1],HEX); 
  Serial.println(hexByte[2],HEX); 
  Serial.println(hexByte[3],HEX); 
  */
  //digitalWrite(TFT_BL,0);
  ///////////////////////////// MY OWN CUSTOM BRIGHTNESS SHIT

  //tft.writecommand(ST7789_WRCTRLD);
  /*
    WRCTRLD    

    b1(d7) = 0
    b2(d6) = 0
    b3(d5) = 0  // bright crtl on/off
    b4(d4) = 0
    b5(d3) = 0  // display dimming on/off
    b6(d2) = 0  // back light control on/off ( digitalWrite(TFT_BL,0) need to turn off i think
    b7(d1) = 0
    b8(d0) = 0
  */
  //tft.writedata( 0b00101100 ); 
  //tft.writedata( 0b00000000 ); 
    
  //writedata( MASK_CTRLD_BCTRL | MASK_CTRLD_DD | MASK_CTRLD_BL );

  tft.writecommand(ST7789_WRCACE); // write content adap bright
 /*
    WRCACE    

    b1(d7) = 0  // CECTRL (Color Enhancement Enable/Disable)
    b2(d6) = 0  
    b3(d5) = 0  CE1  
    b4(d4) = 0  CE0
    b5(d3) = 0  
    b6(d2) = 0  
    b7(d1) = 0  C1
    b8(d0) = 0  C0


    CE1 CE0
    0  0   Low Enhancement
    0  1   Medium
    1  1   High

    C1 C0
    0  0   Off
    0  1   User Interface
    1  0   Still Picture
    1  1   Moving Picture 



  */

  tft.writedata(0b10110011); 


  //uint8_t rdctrld = tft.readcommand8(ST7789_RDCTRLD);
  //Serial.println(rdctrld,BIN);

  //uint16_t rdcace = tft.readcommand16(ST7789_RDCACE);
  //Serial.println(rdcace,BIN);

  tft.writecommand(ST7789_FRCTR2);
  /*
  val framerate
  00 119
  01 111
  02 105
  03 99
  04 94
  05 90
  0f 60
  15 50
  1E 40
  1F 39
  */
  tft.writedata(0x1f);
  

  // ********************** BRIGHTNESS LCD TFT_BL *************************
  //ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  //ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  //ledcAnalogWrite(LEDC_CHANNEL_0, 8);


  //uint8_t rdcabcmb = tft.readcommand8(ST7789_RDCABCMB);
  //Serial.println(rdcabcmb,BIN);
  //uint16_t rdcabcmb = tft.readcommand16(ST7789_RDCABCMB);
  //Serial.println(rdcabcmb,BIN);

  //tft.writecommand(ST7789_WRCABCMB); // Set Content Adap minimum brightness
  //tft.writedata(0x00);

  //tft.writecommand(ST7789_WRDISBV); // Set brightness
  //tft.writedata(0xFF); // 0-255 brightness



}
// ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP ***** ***** SETUP *****









// ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP *****
void loop()
{

  bbutton1.update();
  bbutton2.update();
  server.handleClient();

  bool button1 = false;
  bool button2 = false;
  button1 = (bbutton1.fallingEdge())?true:false;
  button2 = (bbutton2.fallingEdge())?true:false;
  /*
  int button1 = (digitalRead(BUTTON1))?0:1;
  int button2 = (digitalRead(BUTTON2))?0:1;
  */



  ms = millis();

  if( button1 || button2 ) {
    sprintf(buffer,"b1:%d b2:%d",button1,button2);
    Serial.println(buffer);
    if( button2 && (ms-lastButton2Hit) < 2000 )  {
      if( button2HitsInLast2Seconds > 5 ) {
          Serial.println("click bt2 5x");

      }
    }
    if( !display_on ) {
      displayOn();
      return;
    }
  }



  if( button1 ) {

    if( !menu_on ) {
      menu_on = true;
      draw_menu = true;
      return;
    }

    if( menu_on ) {
      currentMenuSelection++;
      if( currentMenuSelection == currentMenuSize ) currentMenuSelection = 0;
      draw_menu = true;
    }
    //display_on?displayOff():displayOn();


  /*
  // MAIN MENU BUTTON
    if( currentMenuEvent == &mainMenuEvent ) {

      (*currentMenuEvent)(BTN1,btn1);

      menuSelected  = 0;
      menuSelection = 1;
      (*currentMenuEvent)(DISPCTRL,0);

    } else {

      // SUB MENU BUTTON AND IS LAST ENTRY
      if( currentMenu->selection() == currentMenu->size()-1  ) {
        // go back if last entry
        Serial.println("Back Back Back");
        currentMenu = &gfxMainMenu;
        currentMenuEvent = &mainMenuEvent;
        currentMenuItems = mainMenu;
        dispPct = 0;


  */
    
    button1 = false;

  }

  if( button2 ) lastButton2Hit = millis();
  
  if( button2  ) {
    if( record ) stopRecord();
    if( menu_on ) (*currentMenuEvent)(0,0);
    return; 
  }

  if(read_mic_on) i2s_read_mic();
  if(display_on) tft_draw();

  if( a2dp_started ) {
    //monoOut.setData(rawBuffer16,samplesRead);
    monoOut.setData(rawBuffer16,samplesRead);
    //SoundData *data = new OneChannelSoundData(rawBuffer16,120);
    if( !a2dp_source.writeData(&monoOut) ) Serial.println("a2dp writeData failed");
  }

  if( !display_on && !read_mic_on && !backlight_on ) {
    delay(1000);
  } else {
    delay(1);
  } 

}
// ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP ***** ***** LOOP *****






uint32_t cmax_threshold = 0;


void i2s_read_mic(void) {
  //esp_err_t i2s_read_ok  = i2s_read(I2S_PORT, &buffer8, sizeof(buffer8), &bytesRead, 100);
  static uint32_t cmax = 0;
  uint32_t maxyz = 0;
  esp_err_t i2s_read_ok  = i2s_read(I2S_PORT, &buffer8, sizeof(buffer8), &bytesRead, portMAX_DELAY);
 
  if( i2s_read_ok == ESP_OK ) {

    samplesRead = bytesRead / 4;
    
    for (int i=0; i<samplesRead; i++) {
       
        if( i == 0 && record && !started_record ) {
          started_record = true; 
          debug_output = false;
          //vTaskSuspend(task_tft_draw);
          delay(500);
        }
        uint8_t xsb = buffer8[i * 4 + 0];
        uint8_t lsb = buffer8[i * 4 + 1];
        uint8_t mid = buffer8[i * 4 + 2];
        uint8_t msb = buffer8[i * 4 + 3];


        rawBuffer24[i*3+0] = msb;
        rawBuffer24[i*3+1] = mid;
        rawBuffer24[i*3+2] = lsb;



                          //   0       1        2        3 
        uint32_t raw32x = (((((xsb<<8)^lsb)<<8)^mid)<<8)^msb;

                          //   3       2        1        0
        uint32_t raw32m = (((((msb<<8)^mid)<<8)^lsb)<<8)^xsb;
        //int32_t raw24m = (((((xsb<<8)^lsb)<<8)^mid)<<8)^msb;
        //int32_t raw24x = (((((msb<<8)^mid)<<8)^lsb)<<8)^xsb;
                         //  0       1        2
        int32_t raw24x = ((((xsb<<8)^lsb)<<8)^mid);
        int32_t raw24xn = ((((xsb<<8)^lsb)<<8)^mid);
                         //  2       1        0
        int32_t raw24m = ((((mid<<8)^lsb)<<8)^xsb);
                         //  1       2        3
        int32_t raw24l = ((((lsb<<8)^mid)<<8)^msb);

                         //  3        2       1
        int32_t raw24 = ((((msb<<8)^mid)<<8)^lsb);
        //int32_t raw24m = ((((msb<<8)^mid)<<8)^lsb);

        if( raw24 & 0x800000 ) raw24 ^= 0xFF000000;
        if( raw24m & 0x800000 ) raw24m ^= 0xFF000000;
        if( raw24xn & 0x800000 ) raw24xn ^= 0xFF000000;
        if( raw24l & 0x800000 ) raw24l ^= 0xFF000000;
      

        int16_t raw16 = (mid<<8)^lsb;
        if( raw24 & 0x800000 && !(raw16&0x8000) ) raw16 ^= 0x8000;
        if( !(raw24 & 0x800000) && (raw16&0x8000) ) raw16 ^= 0x8000;
        rawBuffer16[i] = raw16;
        ///rawBuffer32[i] = (raw24 * 2);
        rawBuffer32[i] = raw24;
        buffer32[i] = (raw32m*2);
        if(  abs(rawBuffer32[i]) > maxyz ) maxyz = abs(rawBuffer32[i]);

        if( debug_output  ) {
        //if( raw24 > 7000 || raw24 < -7000 ) {
        //if( raw24 > 16000 || raw24 < -16000 ) {
        //if( raw24 > 32700 || raw24 < -32700 ) {
          Serial.println("-------------------------------------");

          sprintf(buffer,"%s|%s|%s|%s = %d (orig 0123)",btoa8m4(xsb),btoa8m(lsb),btoa8m2(mid),btoa8m3(msb),raw24);
          Serial.println(buffer);
          sprintf(buffer,"%s|%s|%s|%s = %d (orig 3210)",btoa8m(msb),btoa8m2(mid),btoa8m3(lsb),btoa8m4(xsb),raw24);
          Serial.println(buffer);
          sprintf(buffer,"         %s|%s = %d (orig 16)",btoa8m2(mid),btoa8m3(lsb),raw16);
          Serial.println(buffer);
          sprintf(buffer,"         %s = %d (raw16x)",btoa16m(raw16),raw16);
          Serial.println(buffer);

          sprintf(buffer,"%s = %d (24x)",btoa32m(raw24x),raw24x);
          Serial.println(buffer);
          sprintf(buffer,"%s = %d (24xn)",btoa32m(raw24xn),raw24xn);
          Serial.println(buffer);
          sprintf(buffer,"%s = %d (24m)",btoa32m(raw24m),raw24m);
          Serial.println(buffer);
          sprintf(buffer,"%s = %d (24l)",btoa32m(raw24l),raw24l);
          Serial.println(buffer);
          sprintf(buffer,"%s = %d (32m)",btoa32m(raw32m),raw32m);
          Serial.println(buffer);
          sprintf(buffer,"%s = %d (32x)",btoa32m(raw32x),raw32x);
          Serial.println(buffer);

          if( raw24 & 0x80000000 ) {
            sprintf(buffer,"%s = %d (negative 24)",btoa32m(raw24),raw24);
            Serial.println(buffer);
          } else {
            sprintf(buffer,"%s = %d (positive 24)",btoa32m(raw24),raw24);
            Serial.println(buffer);
          }
        }
       
      } // for (int i=0; i<samplesRead; i++)

      if(record && started_record ) { 
        //if(  cmax < cmax_threshold ) memset(rawBuffer24,0,(size_t)ESP_NOW_MAX_DATA_LEN*3); 
        memcpy(buffer8,buffer32,sizeof(buffer)); 
        recFile.write(buffer8,sizeof(buffer8));
        totalRecBytes += (ESP_NOW_MAX_DATA_LEN*4);




        //recFile.write(rawBuffer24,(size_t)ESP_NOW_MAX_DATA_LEN*3);
        //totalRecBytes += (ESP_NOW_MAX_DATA_LEN*3);


      }




      /*
      if( record && started_record) {
        recFile.write(rawBuffer24,(size_t)ESP_NOW_MAX_DATA_LEN*3);
        totalRecBytes += (ESP_NOW_MAX_DATA_LEN*3);
        //recFile.write(rawBuffer16,ESP_NOW_MAX_DATA_LEN*2);
        //recFile.write(rawBuffer24,ESP_NOW_MAX_DATA_LEN*3);
        //recFile.write(rawBuffer32,ESP_NOW_MAX_DATA_LEN);
        //recFile.write(buffer8,(size_t)bytesRead);
        //totalRecBytes += bytesRead;
        
      }
      */
  }
}
void displayOff(void) {

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.print(ip);
  delay(3000);

  tft.writecommand(ST7789_SLPIN);
  tft.writecommand(ST7789_DISPOFF);    //Display off
  ledcDetachPin(LED_PIN);
  digitalWrite(TFT_BL,0);
  display_on = false;
  read_mic_on = false;
  backlight_on = false;
  Serial.println("Display OFF");


}

void displayOn(void) {
  tft.writecommand(ST7789_SLPOUT);
  tft.writecommand(ST7789_DISPON);
  digitalWrite(TFT_BL,1);
  display_on = true;
  read_mic_on = true;
  backlight_on = true;
  Serial.println("Display ON");


}

void displaySleep(void) {

  tft.writecommand(ST7789_SLPIN);
  tft.writecommand(ST7789_DISPOFF);    //Display off
  ledcDetachPin(LED_PIN);
  digitalWrite(TFT_BL,0);
  backlight_on = false;
  display_on = false;

}

void startRecord(void) {

  /*
  tft.writecommand(ST7789_SLPIN);
  tft.writecommand(ST7789_DISPOFF);    //Display off
  ledcDetachPin(LED_PIN);
  digitalWrite(TFT_BL,0);
  backlight_on = false;
  display_on = false;
  */
  record = true; 
  menu_on = false;
  draw_menu = false;
  totalRecBytes = 0;
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.print("Recording");
  recFile = SD.open("/rec.wav",FILE_WRITE);
  setWaveHeader(200000);
  recFile.write(header,sizeof(header));
  delay(2000);



}


void stopRecord(void) {

  tft.writecommand(ST7789_SLPOUT);
  tft.writecommand(ST7789_DISPON);
  digitalWrite(TFT_BL,1);
  display_on = true;
  read_mic_on = true;
  backlight_on = true;

  record = false;
  started_record = false;
  debug_output = false;
  tft.fillScreen(TFT_BLACK);
  
  setWaveHeader(totalRecBytes);
  recFile.seek(0);
  recFile.write(header,sizeof(header));
  recFile.close(); 

  tft.setCursor(0, 0, 1);
  sprintf(buffer,"Stopped Recording\n%d bytes written",totalRecBytes);
  Serial.println(buffer);
  tft.print(buffer);

  delay(2000);


}




void tft_draw(void) {
    ms = millis();

    if( menu_on ) {
      displayMenu();
      return;
    }

    static unsigned long last_avg_update = 0;
    static uint32_t avg = 0;
    static uint32_t cmax = 0;
    uint32_t maxyz = 0;
    uint32_t inc_avg = 0;
    uint32_t ttl_avg = 0;
    uint32_t iavg = 0;
    uint32_t iavg_count = 0;
    bool should_draw = false;

    //if( !record && !debug_output ) {
    if( !debug_output ) {
      spr.fillSprite(TFT_BLACK);
      spr.drawFastHLine(0,67,239,TFT_WHITE);
    }
    if( record && started_record ) spr.fillCircle(230,10,5,TFT_RED);

    if( (ms - lastScale) > 1000 && max32_y != MAP_SCALE32_DEFAULT && max32_y > MAP_SCALE32_DEFAULT ) {
      max32_y -= (max32_y - MAP_SCALE32_DEFAULT)/3;
      map32Scale -= (map32Scale - MAP_SCALE32_DEFAULT)/3;
      lastScale = ms;

    } 



    /*
    for (int i=0; i<samplesRead; i++) {
      if(  abs(rawBuffer32[i]) > maxyz ) maxyz = abs(rawBuffer32[i]);
      //if( abs(rawBuffer32[i]) > 4300 ) {
      if( cmax > cmax_threshold ) {
        should_draw = true;
      }
    }
    */
    //if( should_draw  ) 
    for (int i=0; i<samplesRead; i++) {
        
        if( (ms - last_avg_update) > 1000 ) {
          inc_avg += abs(rawBuffer32[i]);
          if( i % 10 ) {
            ttl_avg = inc_avg/10;
            iavg += ttl_avg;
            inc_avg = 0;
            iavg_count++;
          }
        }
        

        if( abs(rawBuffer32[i]) > max32_y ) {
          max32_y = abs(rawBuffer32[i]);
          map32Scale = max32_y * 1.08;
          lastScale = millis();
          //Serial.println(max32_y);
        } 


        if( i > 0 ) {
          unsigned int y1 = map(rawBuffer32[i-1],-map32Scale,map32Scale,135,0);
          unsigned int y2 = map(rawBuffer32[i],-map32Scale,map32Scale,135,0);
                
          unsigned int intensity = abs(map(y1,0,135,255,-255));
          uint16_t color = tft.color565(0,0,~intensity);

          spr.drawLine(i+1,y1-2,i+2,y2-2,tft.color565(0,0,intensity+20));
          spr.drawLine(i+1,y1-1,i+2,y2-1,color);
          spr.drawLine(i-1,y1+1,i-2,y2+1,color);
          spr.drawLine(i,y1,i+1,y2,tft.color565(intensity+80,intensity+80,intensity+80));
        }
    }

    /*
    if( (ms - last_avg_update) > 1000 ) {
      if( iavg_count  > 0 ) avg = iavg/iavg_count; 
      last_avg_update = ms;
      cmax = maxyz;
    }
    */
    /*
    sprintf(buffer,"avg: %d\nmax:%d",avg,cmax);
    spr.setTextSize(2);
    spr.setCursor(0,0,1);
    spr.print(buffer);
    */

   

    spr.pushSprite(0,0);

}

char * getWavFileName( void ) {






void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}






void dumpWaveHeader(byte *h) {

  for( int i = 0; i < 44; i++ ) {
    if( isprint(h[i]) ) {
      sprintf(buffer,"%02X (%c) %s",h[i],h[i],headByteLabels[i]);
    } else {
      sprintf(buffer,"%02X %s",h[i],headByteLabels[i]);
    }
    Serial.println(buffer);
  }


}


void setWaveHeader(int wavSize){
  // http://soundfile.sapp.org/doc/WaveFormat/
  /*
  0         4   ChunkID          Contains the letters "RIFF" in ASCII form
                                 (0x52494646 big-endian form).
  4         4   ChunkSize        36 + SubChunk2Size, or more precisely:
                                 4 + (8 + SubChunk1Size) + (8 + SubChunk2Size)
                                 This is the size of the rest of the chunk 
                                 following this number.  This is the size of the 
                                 entire file in bytes minus 8 bytes for the
                                 two fields not included in this count:
                                 ChunkID and ChunkSize.
  8         4   Format           Contains the letters "WAVE"
                                 (0x57415645 big-endian form).

  The "WAVE" format consists of two subchunks: "fmt " and "data":
  The "fmt " subchunk describes the sound data's format:

  12        4   Subchunk1ID      Contains the letters "fmt "
                                 (0x666d7420 big-endian form).
  16        4   Subchunk1Size    16 for PCM.  This is the size of the
                                 rest of the Subchunk which follows this number.
  20        2   AudioFormat      PCM = 1 (i.e. Linear quantization)
                                 Values other than 1 indicate some 
                                 form of compression.
  22        2   NumChannels      Mono = 1, Stereo = 2, etc.
  24        4   SampleRate       8000, 44100, etc.
  28        4   ByteRate         == SampleRate * NumChannels * BitsPerSample/8
  32        2   BlockAlign       == NumChannels * BitsPerSample/8
                                 The number of bytes for one sample including
                                 all channels. I wonder what happens when
                                 this number isn't an integer?
  34        2   BitsPerSample    8 bits = 8, 16 bits = 16, etc.
            2   ExtraParamSize   if PCM, then doesn't exist
            X   ExtraParams      space for extra parameters

  The "data" subchunk contains the size of the data and the actual sound:

  36        4   Subchunk2ID      Contains the letters "data"
                                 (0x64617461 big-endian form).
  40        4   Subchunk2Size    == NumSamples * NumChannels * BitsPerSample/8
                                 This is the number of bytes in the data.
                                 You can also think of this as the size
                                 of the read of the subchunk following this 
                                 number.
  44        *   Data             The actual sound data.
  */
/*
  char cid[4];    // RIFF
  uint32_t fsize;
  char format[4]; // WAVE
  char scid1[4];    // 'fmt '
  uint32_t scid1_size;
  uint16_t audio_format; // 1 = pcm
  uint16_t channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align; // 8,16,32 num_chan*bpp/8
  uint16_t bits_per_sample; // 8, 16,i24, 32
  char scid2[4];  // data
  uint32_t scid2_size;
*/


  waveHeader wavHead = { 
    'R','I','F','F', 
    wavSize + HEADER_SIZE - 8,
    'W','A','V','E', 
    //"WAVE",
    'f','m','t',' ', 
    //"fmt ",
    16,
    1,
    1,
    WAV_SAMPLE_RATE,
    (WAV_SAMPLE_RATE * WAV_NUM_CHAN)*WAV_BYTE_PER_SAMPLE, //SampleRate * NumChannels * BitsPerSample/8 aka BYTE RATE
    WAV_BYTE_PER_SAMPLE*WAV_NUM_CHAN, //NumChannels * BitsPerSample/8 aka BLOCK_ALIGN
    WAV_BIT_PER_SAMPLE,
    'd','a','t','a', 
    //"data",
    wavSize
  };    




  memcpy(&header,&wavHead,44);
  /*
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + HEADER_SIZE - 8;
  header[4] = (byte)(fileSize & 0xFF);            // subchunk1size =...
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10; // subchunk 16?
  header[17] = 0x00; // subchunk
  header[18] = 0x00; // subchunk
  header[19] = 0x00; // subchunk

  header[20] = 0x01; // audio format 1=pcm
  header[21] = 0x00; // audio format
  header[22] = 0x01; // num channels =1
  header[23] = 0x00; // num channels

  header[24] = 0x80; // samplerate 00003E80 = 16000hz 16khz// WAV_SAMPLE_RATE
  header[25] = 0x3E; // samplerate 
  header[26] = 0x00; // samplerate
  header[27] = 0x00; // samplerate

  header[28] = 0x00; // byterate
  header[29] = 0x7D; // byterate
  header[30] = 0x00; // byterate
  header[31] = 0x00; // byterate

  header[32] = 0x02; // block align
  header[33] = 0x00; // block align
  header[34] = 0x10; // bits per sample ( 16 bits
  header[35] = 0x00; // bits per sample

  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);       //subchunk size 2
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
  */


}

void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
}



void tasktft_draw(void *arg) {

  while( 1 ) { 
    tft_draw();
  }

  vTaskDelete(NULL);

}



void i2s_init_mic() {

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX ),
      //.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      //.sample_rate = 8000, // or 44100 if you like
      .sample_rate = WAV_SAMPLE_RATE, // or 44100 if you like
      //.sample_rate = 16000, // or 44100 if you like
      //.sample_rate = 22500, // or 44100 if you like
      //.sample_rate = 32000, // or 44100 if you like
      //.sample_rate = 44100, // or 44100 if you like
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      //.bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
      //.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
      //.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      //.dma_buf_count = 4,
      .dma_buf_len = ESP_NOW_MAX_DATA_LEN * 4,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      //.tx_desc_auto_clear = true,
      .fixed_mclk = 0,
  };


  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  //REG_SET_BIT(I2S_PDM_CONF_REG(I2S_PORT), I2S_PDM2PCM_CONV_EN);
  //REG_SET_BIT(I2S_PDM_CONF_REG(I2S_PORT), I2S_RX_PDM_EN);
  //esp_err_t err = i2s_set_pdm_rx_down_sample(I2S_PORT,I2S_PDM_DSR_16S);

  //esp_err_t err = i2s_set_pdm_rx_down_sample(I2S_PORT,I2S_PDM_DSR_8S);
  //esp_err_t err = i2s_set_pdm_rx_down_sample(I2S_PORT,I2S_PDM_DSR_MAX);
//    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
//    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
    // REG_GET_BIT
    // REG_CLR_BIT


  const i2s_pin_config_t pin_config = {
      .bck_io_num = MIC_I2S_SCK,   // SCK Bit Clock. (14 esp32 thing, hspi_clk) ( ttgo 2 ) 
      .ws_io_num = MIC_I2S_WS,    // WS Word Select aka left/right clock aka LRCL. (15 esp32 thing, hspi_cs0) (ttgo 15 or 21 ?)
      .data_out_num = -1,
      .data_in_num = MIC_I2S_SD,  // SD Data-out of the mic. (someone used 23 on forums). (34 esp32 thing random analog pin) ( ttgo 36)
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}









//lsb to msb
char * btoa8(int8_t n) {
    static char buff[9];
    buff[8] = 0;

    buff[0] = (n&0x1)?'1':'0'; 
    buff[1] = (n&0x2)?'1':'0'; 
    buff[2] = (n&0x4)?'1':'0'; 
    buff[3] = (n&0x8)?'1':'0'; 
    buff[4] = (n&0x10)?'1':'0'; 
    buff[5] = (n&0x20)?'1':'0'; 
    buff[6] = (n&0x40)?'1':'0'; 
    buff[7] = (n&0x80)?'1':'0'; 

    return(buff);
}

char * btoa8m(int8_t n) {
    static char buff[9];
    buff[8] = 0;

    buff[7] = (n&0x1)?'1':'0'; 
    buff[6] = (n&0x2)?'1':'0'; 
    buff[5] = (n&0x4)?'1':'0'; 
    buff[4] = (n&0x8)?'1':'0'; 
    buff[3] = (n&0x10)?'1':'0'; 
    buff[2] = (n&0x20)?'1':'0'; 
    buff[1] = (n&0x40)?'1':'0'; 
    buff[0] = (n&0x80)?'1':'0'; 

    return(buff);
    // 1000 0000 = 0x80
    // 0100 0000 = 0x40
    // 0010 0000 = 0x20
    // 0001 0000 = 0x10
    // 0000 1000 = 0x08
    // 0000 0100 = 0x04
    // 0000 0010 = 0x02
    // 0000 0001 = 0x01

}

char * btoa8m2(int8_t n) {
    static char buff[9];
    buff[8] = 0;

    buff[7] = (n&0x1)?'1':'0'; 
    buff[6] = (n&0x2)?'1':'0'; 
    buff[5] = (n&0x4)?'1':'0'; 
    buff[4] = (n&0x8)?'1':'0'; 
    buff[3] = (n&0x10)?'1':'0'; 
    buff[2] = (n&0x20)?'1':'0'; 
    buff[1] = (n&0x40)?'1':'0'; 
    buff[0] = (n&0x80)?'1':'0'; 

    return(buff);
}

char * btoa8m3(int8_t n) {
    static char buff[9];
    buff[8] = 0;

    buff[7] = (n&0x1)?'1':'0'; 
    buff[6] = (n&0x2)?'1':'0'; 
    buff[5] = (n&0x4)?'1':'0'; 
    buff[4] = (n&0x8)?'1':'0'; 
    buff[3] = (n&0x10)?'1':'0'; 
    buff[2] = (n&0x20)?'1':'0'; 
    buff[1] = (n&0x40)?'1':'0'; 
    buff[0] = (n&0x80)?'1':'0'; 

    return(buff);

}

char * btoa8m4(int8_t n) {
    static char buff[9];
    buff[8] = 0;

    buff[7] = (n&0x1)?'1':'0'; 
    buff[6] = (n&0x2)?'1':'0'; 
    buff[5] = (n&0x4)?'1':'0'; 
    buff[4] = (n&0x8)?'1':'0'; 
    buff[3] = (n&0x10)?'1':'0'; 
    buff[2] = (n&0x20)?'1':'0'; 
    buff[1] = (n&0x40)?'1':'0'; 
    buff[0] = (n&0x80)?'1':'0'; 

    return(buff);

}





//lsb to msb
char * btoa16(int16_t n) {
    static char buff[17];
    buff[16] = 0;

    //Right side of bin num
    buff[0] = (n&0x1)?'1':'0'; 
    buff[1] = (n&0x2)?'1':'0'; 
    buff[2] = (n&0x4)?'1':'0'; 
    buff[3] = (n&0x8)?'1':'0'; 
    buff[4] = (n&0x10)?'1':'0'; 
    buff[5] = (n&0x20)?'1':'0'; 
    buff[6] = (n&0x40)?'1':'0'; 
    buff[7] = (n&0x80)?'1':'0'; 

    buff[8] = (n&0x100)?'1':'0'; 
    buff[9] = (n&0x200)?'1':'0'; 
    buff[10] = (n&0x400)?'1':'0'; 
    buff[11] = (n&0x800)?'1':'0'; 
    buff[12] = (n&0x1000)?'1':'0'; 
    buff[13] = (n&0x2000)?'1':'0'; 
    buff[14] = (n&0x4000)?'1':'0'; 
    buff[15] = (n&0x8000)?'1':'0'; 

    return(buff);
}

char * btoa16m(int16_t n) {
    static char buff[17];
    buff[16] = 0;

    //Right side of bin num
    buff[15] = (n&0x1)?'1':'0'; 
    buff[14] = (n&0x2)?'1':'0'; 
    buff[13] = (n&0x4)?'1':'0'; 
    buff[12] = (n&0x8)?'1':'0'; 
    buff[11] = (n&0x10)?'1':'0'; 
    buff[10] = (n&0x20)?'1':'0'; 
    buff[9] = (n&0x40)?'1':'0'; 
    buff[8] = (n&0x80)?'1':'0'; 

    buff[7] = (n&0x100)?'1':'0'; 
    buff[6] = (n&0x200)?'1':'0'; 
    buff[5] = (n&0x400)?'1':'0'; 
    buff[4] = (n&0x800)?'1':'0'; 
    buff[3] = (n&0x1000)?'1':'0'; 
    buff[2] = (n&0x2000)?'1':'0'; 
    buff[1] = (n&0x4000)?'1':'0'; 
    buff[0] = (n&0x8000)?'1':'0'; 

    return(buff);
}

//lsb to msb
char * btoa32(int32_t n) {
    static char buff[33];
    buff[32] = 0;

    //Right side of bin num
    buff[0] = (n&0x1)?'1':'0'; 
    buff[1] = (n&0x2)?'1':'0'; 
    buff[2] = (n&0x4)?'1':'0'; 
    buff[3] = (n&0x8)?'1':'0'; 
    buff[4] = (n&0x10)?'1':'0'; 
    buff[5] = (n&0x20)?'1':'0'; 
    buff[6] = (n&0x40)?'1':'0'; 
    buff[7] = (n&0x80)?'1':'0'; 

    buff[8] = (n&0x100)?'1':'0'; 
    buff[9] = (n&0x200)?'1':'0'; 
    buff[10] = (n&0x400)?'1':'0'; 
    buff[11] = (n&0x800)?'1':'0'; 
    buff[12] = (n&0x1000)?'1':'0'; 
    buff[13] = (n&0x2000)?'1':'0'; 
    buff[14] = (n&0x4000)?'1':'0'; 
    buff[15] = (n&0x8000)?'1':'0'; 

    buff[16] = (n&0x10000)?'1':'0'; 
    buff[17] = (n&0x20000)?'1':'0'; 
    buff[18] = (n&0x40000)?'1':'0'; 
    buff[19] = (n&0x80000)?'1':'0'; 
    buff[20] = (n&0x100000)?'1':'0'; 
    buff[21] = (n&0x200000)?'1':'0'; 
    buff[22] = (n&0x400000)?'1':'0'; 
    buff[23] = (n&0x800000)?'1':'0'; 

    buff[24] = (n&0x1000000)?'1':'0'; 
    buff[25] = (n&0x2000000)?'1':'0'; 
    buff[26] = (n&0x4000000)?'1':'0'; 
    buff[27] = (n&0x8000000)?'1':'0'; 
    buff[28] = (n&0x10000000)?'1':'0'; 
    buff[29] = (n&0x20000000)?'1':'0'; 
    buff[30] = (n&0x40000000)?'1':'0'; 
    buff[31] = (n&0x80000000)?'1':'0'; 

    return(buff);
}

char * btoa32m(int32_t n) {
    static char buff[33];
    buff[32] = 0;

    //Right side of bin num
    buff[0] = (n&0x80000000)?'1':'0'; 
    buff[1] = (n&0x40000000)?'1':'0'; 
    buff[2] = (n&0x20000000)?'1':'0'; 
    buff[3] = (n&0x10000000)?'1':'0'; 
    buff[4] = (n&0x8000000)?'1':'0'; 
    buff[5] = (n&0x4000000)?'1':'0'; 
    buff[6] = (n&0x2000000)?'1':'0'; 
    buff[7] = (n&0x1000000)?'1':'0'; 

    buff[8] = (n&0x800000)?'1':'0'; 
    buff[9] = (n&0x400000)?'1':'0'; 
    buff[10] = (n&0x200000)?'1':'0'; 
    buff[11] = (n&0x100000)?'1':'0'; 
    buff[12] = (n&0x80000)?'1':'0'; 
    buff[13] = (n&0x40000)?'1':'0'; 
    buff[14] = (n&0x20000)?'1':'0'; 
    buff[15] = (n&0x10000)?'1':'0'; 

    buff[16] = (n&0x8000)?'1':'0'; 
    buff[17] = (n&0x4000)?'1':'0'; 
    buff[18] = (n&0x2000)?'1':'0'; 
    buff[19] = (n&0x1000)?'1':'0'; 
    buff[20] = (n&0x800)?'1':'0'; 
    buff[21] = (n&0x400)?'1':'0'; 
    buff[22] = (n&0x200)?'1':'0'; 
    buff[23] = (n&0x100)?'1':'0'; 

    buff[24] = (n&0x80)?'1':'0'; 
    buff[25] = (n&0x40)?'1':'0'; 
    buff[26] = (n&0x20)?'1':'0'; 
    buff[27] = (n&0x10)?'1':'0'; 
    buff[28] = (n&0x8)?'1':'0'; 
    buff[29] = (n&0x4)?'1':'0'; 
    buff[30] = (n&0x2)?'1':'0'; 
    buff[31] = (n&0x1)?'1':'0'; 

    return(buff);
}

char * btoa24(int32_t n) {
    static char buff[25];
    buff[24] = 0;

    //Right side of bin num
    /*
    buff[0] = (n&0x1)?'1':'0'; 
    buff[1] = (n&0x2)?'1':'0'; 
    buff[2] = (n&0x4)?'1':'0'; 
    buff[3] = (n&0x8)?'1':'0'; 
    buff[4] = (n&0x10)?'1':'0'; 
    buff[5] = (n&0x20)?'1':'0'; 
    buff[6] = (n&0x40)?'1':'0'; 
    buff[7] = (n&0x80)?'1':'0'; 
    */

    buff[0] = (n&0x100)?'1':'0'; 
    buff[1] = (n&0x200)?'1':'0'; 
    buff[2] = (n&0x400)?'1':'0'; 
    buff[3] = (n&0x800)?'1':'0'; 
    buff[4] = (n&0x1000)?'1':'0'; 
    buff[5] = (n&0x2000)?'1':'0'; 
    buff[6] = (n&0x4000)?'1':'0'; 
    buff[7] = (n&0x8000)?'1':'0'; 

    buff[8] = (n&0x10000)?'1':'0'; 
    buff[9] = (n&0x20000)?'1':'0'; 
    buff[10] = (n&0x40000)?'1':'0'; 
    buff[11] = (n&0x80000)?'1':'0'; 
    buff[12] = (n&0x100000)?'1':'0'; 
    buff[13] = (n&0x200000)?'1':'0'; 
    buff[14] = (n&0x400000)?'1':'0'; 
    buff[15] = (n&0x800000)?'1':'0'; 

    buff[16] = (n&0x1000000)?'1':'0'; 
    buff[17] = (n&0x2000000)?'1':'0'; 
    buff[18] = (n&0x4000000)?'1':'0'; 
    buff[19] = (n&0x8000000)?'1':'0'; 
    buff[20] = (n&0x10000000)?'1':'0'; 
    buff[21] = (n&0x20000000)?'1':'0'; 
    buff[22] = (n&0x40000000)?'1':'0'; 
    buff[23] = (n&0x80000000)?'1':'0'; 

    return(buff);
}

char * btoa24m(int32_t n) {
    static char buff[25];
    buff[24] = 0;

    //Right side of bin num
    buff[23] = (n&0x1)?'1':'0'; 
    buff[22] = (n&0x2)?'1':'0'; 
    buff[21] = (n&0x4)?'1':'0'; 
    buff[20] = (n&0x8)?'1':'0'; 
    buff[19] = (n&0x10)?'1':'0'; 
    buff[18] = (n&0x20)?'1':'0'; 
    buff[17] = (n&0x40)?'1':'0'; 
    buff[16] = (n&0x80)?'1':'0'; 

    buff[15] = (n&0x100)?'1':'0'; 
    buff[14] = (n&0x200)?'1':'0'; 
    buff[13] = (n&0x400)?'1':'0'; 
    buff[12] = (n&0x800)?'1':'0'; 
    buff[11] = (n&0x1000)?'1':'0'; 
    buff[10] = (n&0x2000)?'1':'0'; 
    buff[9] = (n&0x4000)?'1':'0'; 
    buff[8] = (n&0x8000)?'1':'0'; 

    buff[7] = (n&0x10000)?'1':'0'; 
    buff[6] = (n&0x20000)?'1':'0'; 
    buff[5] = (n&0x40000)?'1':'0'; 
    buff[4] = (n&0x80000)?'1':'0'; 
    buff[3] = (n&0x100000)?'1':'0'; 
    buff[2] = (n&0x200000)?'1':'0'; 
    buff[1] = (n&0x400000)?'1':'0'; 
    buff[0] = (n&0x800000)?'1':'0'; 

    Serial.println(buff);

    return(buff);
}

int32_t invert24bit(int32_t n) {
    int32_t out = 0;
    //Right side of bin num
    if(n&0x1)out^=0x800000; 
    if(n&0x2)out^=0x400000; 
    if(n&0x4)out^=0x200000;
    if(n&0x8)out^=0x100000;
    if(n&0x10)out^=0x80000;
    if(n&0x20)out^=0x40000;
    if(n&0x40)out^=0x20000;
    if(n&0x80)out^=0x10000;

    if(n&0x100)out^=0x8000;
    if(n&0x200)out^=0x4000;
    if(n&0x400)out^=0x2000;
    if(n&0x800)out^=0x1000;
    if(n&0x1000)out^=0x800;
    if(n&0x2000)out^=0x400;
    if(n&0x4000)out^=0x200;
    if(n&0x8000)out^=0x100;

    if(n&0x10000)out^=0x80;
    if(n&0x20000)out^=0x40;
    if(n&0x40000)out^=0x20;
    if(n&0x80000)out^=0x10;
    if(n&0x100000)out^=0x8;
    if(n&0x200000)out^=0x4;
    if(n&0x400000)out^=0x2;
    if(n&0x800000)out^=0x1;

    return(out);

}

//swap the bits in a 32 bit number ( bit 0 to 31, 1 to 30 etc...)
int32_t invert24bitS(int32_t n) {
    int32_t out = 0;
    //Right side of bin num
    if(n&0x1)out^=0x80000000; 
    if(n&0x2)out^=0x40000000; 
    if(n&0x4)out^=0x20000000;
    if(n&0x8)out^=0x10000000;
    if(n&0x10)out^=0x8000000;
    if(n&0x20)out^=0x4000000;
    if(n&0x40)out^=0x2000000;
    if(n&0x80)out^=0x1000000;

    if(n&0x100)out^=0x800000;
    if(n&0x200)out^=0x400000;
    if(n&0x400)out^=0x200000;
    if(n&0x800)out^=0x100000;
    if(n&0x1000)out^=0x80000;
    if(n&0x2000)out^=0x40000;
    if(n&0x4000)out^=0x20000;
    if(n&0x8000)out^=0x10000;

    if(n&0x10000)out^=0x8000;
    if(n&0x20000)out^=0x4000;
    if(n&0x40000)out^=0x2000;
    if(n&0x80000)out^=0x1000;
    if(n&0x100000)out^=0x800;
    if(n&0x200000)out^=0x400;
    if(n&0x400000)out^=0x200;
    if(n&0x800000)out^=0x100;

    return(out);

}

//Display the middle two bytes of a 32 bit 
char * btoa16x(int32_t n) {
    static char buff[17];
    buff[16] = 0;

    buff[0] = (n&0x100)?'1':'0'; 
    buff[1] = (n&0x200)?'1':'0'; 
    buff[2] = (n&0x400)?'1':'0'; 
    buff[3] = (n&0x800)?'1':'0'; 
    buff[4] = (n&0x1000)?'1':'0'; 
    buff[5] = (n&0x2000)?'1':'0'; 
    buff[6] = (n&0x4000)?'1':'0'; 
    buff[7] = (n&0x8000)?'1':'0'; 

    buff[8] = (n&0x10000)?'1':'0'; 
    buff[9] = (n&0x20000)?'1':'0'; 
    buff[10] = (n&0x40000)?'1':'0'; 
    buff[11] = (n&0x80000)?'1':'0'; 
    buff[12] = (n&0x100000)?'1':'0'; 
    buff[13] = (n&0x200000)?'1':'0'; 
    buff[14] = (n&0x400000)?'1':'0'; 
    buff[15] = (n&0x800000)?'1':'0'; 

    return(buff);
}

int16_t map24to16(int32_t x) {
  int32_t in_min = -8388607;
  int32_t in_max =  8388607; 
  int32_t out_min = -32768;
  int32_t out_max =  32768;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


