/*
 * rotMatrix specializes in driving a 16x16 WS2812b matrix.
 * It offers several alternative sources for image data and ways to specify which of them to show.
 * 
 * It currently supports a limited number of images stored in dedicated variables in matrixImg.h 
 * Those 16 pics are sometimes referenced as 'internal'
 * 
 * It supports image data on the mcu's build-in flash, nominally 3 MB though but with fixed block size and 
 * small files the number is the limiting factor (<800).
 * This is sometimes freferenced as 'external'
 * 
 * Finally the sketch supports receiving image data on the fly as a list of rgb values.
 * 
 * In all cases the image data are 256 hex rgb values which describe the 16x16 matrix in a zig-zag way
 * (16 values left-to-right, change of row, 16 values right to left, change of row, repeat)
 * 
 * Regardles where stored the image data need to be shuffled accordingly to render propperly.
 * 
 * The sketch supports 2 different ways to select images and adjust parameters: 
 * - mqtt messages (sort of the native language) and
 * - manual input with a rotary endoder 
 * 
 * The rotary encoder's switch shifts three functions, initially the encoder regulates the brightness, 
 * the next level circles through the 'internal' pics and the last level circles through the 'external' pics.
 * 
 * 
 * 
 * This sketch was developped and tested with arduino ESP8266 core 2.4.2
 * 2.5.0 has occasional but rather frequent crashes in SPIFFS 
 * >2.5.0 has breaking changes in the way interrupts are handled 
 * 
 * Wishlist: 
 * btnClick of rotary still bounces, some sort of optical feedback of the current level would really help
 * 
 * 
 * The Sketch is prepared to keep some config separate for a range of devices which run individual compiles of the sketch, define it below.
 * 
 * The sketch has an infrastructure of connection to wifi with shared secrets, mqtt with shared secrets and Arduino OTA with shared secret.
 * 
 * See help.txt for details on controlling the display
 * //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo)
 *
 */

#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
 
#include "mqtt_ota_secrets.h"
#include "wifi_secrets.h"

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "FastLED.h"
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <Streaming.h>

#include <ESP8266WiFi.h>
#include <FS.h>   //Include File System Headers
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

#include "matrixImg.h"

/*
  008  interne img erneuert
  009  zugriff über index, neue img-data in /data
  010  cycleInt(), cycleExt()
  011  showByName, cycleByName
  012  some obsolete parameters taken away. Documentation
 */

const char* version = "012";


//#define rotaryMatrix true
//#define MEDIMATRIX true
#define MATRIXKANTE true

/*****************************************************************************/

#ifdef rotaryMatrix
  #define SENSORNAME "rotaryMatrix"
  const char* state_topic = "led/rm/state";
  const char* set_topic   = "led/rm/set";
  const char* dbg_topic   = "led/rm/dbg";
  const char* scan_topic  =  "wifiscan/rm";
  byte brightness = 6;
  #define NUM_LEDS    256
  
  bool stateOn = false;
  bool startFade = false;
  
  CHSV baseColor(255,240,255);
  CHSV currColor(15,255,255);
  int hueRange=12;
  int quart=23;
  int direction=1;
  
  int stepTime=60;
  const char* effect = "kuss";
  String effectString= effect;
#endif



#ifdef MATRIXKANTE
#define SENSORNAME "matrixKante"

const char* state_topic = "led/mk/state";
const char* set_topic   = "led/mk/set";
const char* dbg_topic   = "led/mk/dbg";
const char* scan_topic  =  "wifiscan/mk";

byte brightness = 6;

#define NUM_LEDS    256

bool stateOn = true;


CHSV baseColor(141,240,255);
CHSV currColor(15,255,255);
int hueRange=17;
int quart=23;
int direction=1;

int stepTime=60;
const char* effect = "om";
  String effectString= effect;
#endif



#ifdef MEDIMATRIX
#define SENSORNAME "mediMatrix"

const char* state_topic = "led/mm/state";
const char* set_topic   = "led/mm/set";
const char* dbg_topic   = "led/mm/dbg";
const char* scan_topic  =  "wifiscan/mm";

byte brightness = 6;

#define NUM_LEDS    256

bool stateOn = true;


CHSV baseColor(141,240,255);
CHSV currColor(15,255,255);
int hueRange=17;
int quart=23;
int direction=1;

int stepTime=60;
const char* effect = "kreis";
  String effectString= effect;
#endif




#ifdef TESTMATRIX
#define SENSORNAME "testMatrix"

const char* state_topic = "led/tm/state";
const char* set_topic   = "led/tm/set";
const char* dbg_topic   = "led/tm/dbg";
const char* scan_topic  =  "wifiscan/tm";

byte brightness = 6;

#define NUM_LEDS    256

bool stateOn = true;


CHSV baseColor(123,240,255);
CHSV currColor(15,255,255);
int hueRange=17;
int quart=23;
int direction=1;

int stepTime=180;
const char* effect = "delphin";
  String effectString= effect;
#endif



// timed_loop
#define INTERVAL_1   3000
#define INTERVAL_2  30000
#define INTERVAL_3  60000
#define INTERVAL_4 180000

unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;

// extraWurst für colorLoop
unsigned long INTERVAL_0 = 60;
unsigned long time_0     =  0;



// SPIFFS
char filename[50];

char* file_ext = ".dat";
char* path = "/";
int stepFactor = 1;

char* cycleMode = "none";
//char avar[15]={};
char* cycleSteps[]= {"123456789abcde", "223456789abcde", "323456789abcde", "423456789abcde", "523456789abcde", "623456789abcde", "723456789abcde", "823456789abcde", "923456789abcde", "a23456789abcde", "b23456789abcde", "c23456789abcde", "d23456789abcde", "e23456789abcde", "f23456789abcde", "g23456789abcde", "h23456789abcde", "i23456789abcde", "j23456789abcde", "k23456789abcde"};
int cycleVon = 0;
int cycleBis = 1;
int cycleIndex = 0;


// rotary
long oldPosition  = -999;
boolean isButtonPressed = false;
long lastUpdateMillis = 0;

int btnModus = 0;
const char* modi[] = {"brightness","intern","extern"};
int rotaryVals[] = { 6,1,0};

int maxBrightness = 64;



// wifiscan
int scanId=0;
bool scanne=false;

/****************************  **************************************************/

const char* on_cmd = "ON";
const char* off_cmd = "OFF";



// debug messages
const int numMsg=20;
int msgCount=0;
String msg=".";
String arrMessages[numMsg];

long adhoc [256];

/****************************************FOR JSON***************************************/
const int BUFFER_SIZE = JSON_OBJECT_SIZE(25);



/*********************************** FastLED Defintions ********************************/


#define DATA_PIN    2

#define CHIPSET     WS2812B
#define COLOR_ORDER GRB

struct CRGB leds[NUM_LEDS];


byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

byte red = 255;
byte green = 255;
byte blue = 255;

unsigned long  INTERVAL_anim = 2000;
int animFactor = 1;
unsigned long time_anim = 0;


bool firstRun  = true;



/******************************** GLOBALS for animations *******************************/


int loopCount=0;
int animCount=0;
unsigned long lastLoop=millis();

float hueStep;
float hue;
int delayMultiplier = 1;


// instanzen
Encoder myEnc(D5, D6);

WiFiClient espClient;
PubSubClient mqClient(espClient);


/* This code is for executing the interrupt in ESP8266.
*  The main purpose is to solve the ISR not in RAM isssue.
* 
*/
//void ICACHE_RAM_ATTR handleKey ();


void setup() {
  
  Serial.begin(115200);
  debug(String(SENSORNAME)+" "+version,true);

  FastLED.addLeds<CHIPSET, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);

  setupWifi();
  
  setupMq();
  setupOta();

  
  colorInit();
  
  setupSpiff();

  // rotary encoder
  pinMode(D7, INPUT_PULLUP);
  attachInterrupt(D7, handleKey, RISING);
  myEnc.write(brightness*4*-1);


}
void handleKey() {
  isButtonPressed = true;  
  //Serial << "klick\n";
}

/********************************** START MAIN LOOP*****************************************/

void loop() {
  
  mqClient.loop();
  ArduinoOTA.handle();
  rotary_loop();
  
  timed_loop();
} 

void timed_loop() {
   if(millis() > time_0 + INTERVAL_0){
    time_0 = millis();

      colorLoop();

  } 
  
  if(millis() > time_1 + INTERVAL_1){
    time_1 = millis();
    
     
    if (!mqClient.connected()) {
      mqConnect();
    }  
    checkDebug();       
    
  }
   
  if(millis() > time_2 + INTERVAL_2){
    time_2 = millis();
 
    
          

  }
 
  if(millis() > time_3 + INTERVAL_3){
    time_3 = millis();

  }

  if(millis() > time_4 + INTERVAL_4){
    time_4 = millis();
        if (scanne) {
          scanWifi();
        }            
  }


}

void checkCycle(){
  if (cycleMode == "cycleExt"){
    setFilenameByIndex(cycleIndex);
    readImgData ();
    cycleIndex++;
    if (cycleIndex > cycleBis){cycleIndex = cycleVon;};
  }
  else if (cycleMode == "cycleInt"){
    selectEffect(cycleIndex);
    cycleIndex++;
    if (cycleIndex > cycleBis){cycleIndex = cycleVon;};
  }
  
  else if (cycleMode == "cycleByName"){
   //String fn=String(cycleSteps[cycleIndex]);
    setFilenameByName(cycleSteps[cycleIndex]);
    readImgData ();
    cycleIndex++;
    if (cycleIndex > cycleBis){cycleIndex = cycleVon;};

//    String dbgLine= String(fn)+" cycleIndex: "+String(cycleIndex)+" cycleVon: "+String(cycleVon)+" cycleBis: "+String(cycleBis);
//    debug(dbgLine,false);

//    dbgLine="";
//    for (int i=cycleVon;i<cycleBis;i++){
//      dbgLine += String(cycleSteps[i]);
//      dbgLine += " ";
//    }
//    debug(dbgLine,false);
  }
}

/********************************** START Set Color*****************************************/
void setColor(int inR, int inG, int inB) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].red   = inR;
    leds[i].green = inG;
    leds[i].blue  = inB;
  }


  FastLED.setBrightness(brightness);
  FastLED.show();

}


void colorInit(){
  effectString=effect;
  hueStep = hueRange*1.0 / quart;

 for (int i = 0; i < NUM_LEDS; i++) {
    currColor.h = ( baseColor.h+i);
    leds[i]=currColor;
  }

  Serial << F("colorInit brightness: ")<< brightness<< F("\n");
  FastLED.setBrightness(brightness);
  FastLED.setDither( 0 );
  FastLED.show();
}

void fillHsv(CHSV hsv){
  for (int i = 0; i < NUM_LEDS; i++) {
    hsv2rgb_rainbow( hsv, leds[i]);
  }
  FastLED.setBrightness(brightness);
  //FastLED.show();    
}


void showleds(){

  delay(1);

  if (stateOn) {
    FastLED.setBrightness(brightness);  //EXECUTE EFFECT COLOR
    FastLED.show();
    if (stepTime > 0 && stepTime < 250) {  //Sets animation speed based on receieved value
      FastLED.delay(stepTime / 10 * delayMultiplier); //1000 / transitionTime);
      //delay(10*transitionTime);
    }
  }
}

/**********************************      *****************************************/


uint32_t hex2int(char *hex) {
    uint32_t val = 0;
    while (*hex) {
        // get current character then increment
        char byte = *hex++; 
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
        // shift 4 to make space for new digit, and add the 4 bits of the new digit 
        val = (val << 4) | (byte & 0xF);
    }
    val -= 134217728;
    return val;
}




void rotary_loop() {
  
  // newPosition : Wert aus myEnc : ViererSchritte auf Einser reduzieren : Uhrzeigersinn incrementiert : * stepFactor
  
  long newPosition = myEnc.read()/4 * -1 * stepFactor;
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    //getDataByIndex (newPosition);
    doNewRotaryVal(newPosition);
  }
  // software debounce
  if (isButtonPressed && millis() - lastUpdateMillis > 500) {
    isButtonPressed = false;
    lastUpdateMillis = millis();

   doButtonKlick();
  }
}

void doNewRotaryVal(int newPosition){
  Serial << F("rotary: ") << (newPosition) << F("\n");
  if (btnModus==0) { setBrightness(newPosition); } else
  if (btnModus==1) { selectEffect(newPosition); } 
  else if (btnModus==2) { getDataByIndex(newPosition); } 
}

void doButtonKlick(){
  Serial.println("Tick");
  // rotaryVal des bisherigen Modus sichern
  rotaryVals[btnModus] = myEnc.read();

  // Modus wechseln
  btnModus++; 
  if (btnModus > 2 ){btnModus=0;}
  
  // rotary mit entsprechendem Wert einstellen
  oldPosition = rotaryVals[btnModus];
  myEnc.write(oldPosition);

  // status kommunizieren
  Serial << modi[btnModus]<< F("\n");
}



/*
 * Schaltet zwischen Modi btnModus
 * 0: Helligkeit
 * 1: images aus intern
 * 2: images aus spiffs getDataByIndex
 */

//
//void setStepFactor(){
//  if (stepFactor==1){stepFactor=10;}
//  else if (stepFactor==10){stepFactor=100;}
//  else if (stepFactor==100){stepFactor=1;}
//}

void setBrightness(int pos ){
  if (pos < 1) {pos= maxBrightness;}
  if (pos > maxBrightness) {pos= 1;}
  brightness = pos;
  FastLED.setBrightness(brightness);
}

/*
  das ist so ausgelegt auf imagedaten in stetiger Reihe 1.dat, 2.dat, 3.dat, 4.dat, ...
  stattdessen braucht es tatsächlich einen index der dateinamen so dass fn=index[ndx]

  weiter soll dies nur den Dateinamen ermitteln und einlesen in andere funktion.
*/


void setFilenameByName (char* fn) {

  strcpy (filename,path);
  strcat(filename,fn);
  strcat(filename,file_ext);

  Serial.println(filename);   
}

void setFilenameByIndex (int ndx) {
const  char* fn = findex[ndx];
  char buffer[32];
  //sprintf(buffer, "%d", fn);
  strcpy (filename,path);
  strcat(filename,fn);
  strcat(filename,file_ext);

  Serial.println(filename);   
}

void readImgData (){

  if (SPIFFS.exists(filename)){
    //Read File data
    File f = SPIFFS.open(filename, "r");
    
    if (!f) {
      Serial.println("file open failed");
    }
    else
    {
        String chunkVal = "";    
        char separator=',';
        char cr;
        int j = 0;

      
        Serial.println("Reading Data from File:");
        //Data from file
        for(int i=0;i<f.size();i++) //Read upto complete file size
        {
          cr = (char)f.read();

          if (cr != separator){
                chunkVal.concat(cr);
          }
          else
          {
             chunkVal.trim();
             int anz=chunkVal.length();
             
             char hex[anz+1];
             strcpy(hex,chunkVal.c_str());
             long test = hex2int(hex);
             
             if (chunkVal != ""){
               adhoc[j] = test;

              // Serial << j << F(": ") << chunkVal<< F("->")  << adhoc[j] << F("\n") ;
               chunkVal = "";
               j++;
             }
                   
          }
          
          
        }
        f.close();  //Close file
        Serial.println("File Closed");
       firstRun=true;
       effect="adhoc";
       effectString=effect;
    }
  } else { Serial << F("File not found\n "); }  
}


void getDataByIndex (int ndx) {
 ndx = ndx % findexEntryCnt;
 
 const  char* fn = findex[ndx];
 
  strcpy (filename,path);
  strcat(filename,fn);
  strcat(filename,file_ext);

  Serial.println(filename);   


  if (SPIFFS.exists(filename)){
    //Read File data
    File f = SPIFFS.open(filename, "r");
    
    if (!f) {
      Serial.println("file open failed");
    }
    else
    {
        String chunkVal = "";    
        char separator=',';
        char cr;
        int j = 0;

      
        Serial.println("Reading Data from File:");
        //Data from file
        for(int i=0;i<f.size();i++) //Read upto complete file size
        {
          cr = (char)f.read();

          if (cr != separator){
                chunkVal.concat(cr);
          }
          else
          {
             chunkVal.trim();
             int anz=chunkVal.length();
             
             char hex[anz+1];
             strcpy(hex,chunkVal.c_str());
             long test = hex2int(hex);
             
             if (chunkVal != ""){
               adhoc[j] = test;

              // Serial << j << F(": ") << chunkVal<< F("->")  << adhoc[j] << F("\n") ;
               chunkVal = "";
               j++;
             }
                   
          }
          
          
        }
        f.close();  //Close file
        Serial.println("File Closed");
       firstRun=true;
       effect="adhoc";
       effectString=effect;
    }
  } else { Serial << F("File not found\n "); }
  //interrupts();
}




///////////////////////////////////////////////////////////////////////////////////////

void selectEffect(int selector){
  int numVariants = 16;
  if (selector < 0){selector *=-1;}
  int sel = selector % numVariants;
  effect = "solid";
  
  switch(sel){
    case 0: effect = "om"; break;
    case 1: effect = "danke"; break;
    case 2: effect = "herz"; break;
    case 3: effect = "kreis"; break;
    case 4: effect = "kuss"; break;
    case 5: effect = "schmetterling"; break;
    case 6: effect = "delphin"; break;
    case 7: effect = "maskulin"; break;
    case 8: effect = "feminin"; break;
    case 9: effect = "regenbogen"; break;
    case 10: effect = "tatzen"; break;   
    case 11: effect = "cool"; break; 
    case 12: effect = "apfel"; break; 
    case 13: effect = "banane"; break;
    case 14: effect = "erdbeer"; break;
    case 15: effect = "melone"; break; 

  }

        firstRun=true;
      effectString=effect;
      Serial << sel <<F(": ")<< effect << "\n";
}

//////////////////////////////////////////////////////////////////////

void colorLoop(){
  

    if (effectString == "rangeWave"){
      rangeWave();
    }
    else if (effectString == "shiftBand"){
      shiftBand();
    }
    else if (effectString == "sineHue"){
      sineHue();
    }

    else if (effectString == "om"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = om[i];  }  firstRun=false; }          
    }    

    else if (effectString == "danke"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = danke[i];  }  firstRun=false; }          
    }    

    else if (effectString == "herz"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = herz[i];  }  firstRun=false; }          
    }    

    else if (effectString == "kreis"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = kreis[i];  }  firstRun=false; }          
    }    

    else if (effectString == "kuss"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = kuss[i];  }  firstRun=false; }          
    }    

   else if (effectString == "schmetterling"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = schmetterling[i];  }  firstRun=false; }          
    }    

    else if (effectString == "delphin"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = delphin[i];  }  firstRun=false; }          
    }    

    else if (effectString == "feminin"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = feminin[i];  }  firstRun=false; }          
    }    

    else if (effectString == "maskulin"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = maskulin[i];  }  firstRun=false; }          
    }    
   else if (effectString == "regenbogen"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = regenbogen[i];  }  firstRun=false; }          
    }    

    else if (effectString == "tatzen"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = tatzen[i];  }  firstRun=false; }          
    }    

    else if (effectString == "cool"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = cool[i];  }  firstRun=false; }          
    }    

    else if (effectString == "apfel"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = apfel[i];  }  firstRun=false; }          
    }    


    else if (effectString == "banane"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = banane[i];  }  firstRun=false; }          
    }    

    else if (effectString == "erdbeer"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = erdbeer[i];  }  firstRun=false; }          
    }    

    else if (effectString == "melone"){
       if (firstRun){  for(int i = 0; i < NUM_LEDS; i++) { leds[i] = melone[i];  }  firstRun=false; }          
    }    

 



    
    else if (effectString == "adhoc"){
       if (firstRun){
        for(int i = 0; i < NUM_LEDS; i++) {
          leds[i] = adhoc[i];  
        }
         
         firstRun=false;
       }     
      
    }     
   
    else if (effectString == "solid"){
      solid();
    }    
    else {
      debug("effect not known: |"+effectString+"|",true);
    }
    //showleds();
    FastLED.show();
    


  checkCycle();
  INTERVAL_0 = stepTime * delayMultiplier * animFactor;
}

/////////////////////////////////////////////////////////


void rangeWave(){
    unsigned long now = millis();
//    Serial << (now - lastLoop) << " " << stepTime << "\n";
//    delay(1000);
    if (now - lastLoop > stepTime) {
        lastLoop = now;
        loopCount++;
        loopCount = loopCount % NUM_LEDS;
        
        hue = baseColor.h*1.0;
        hueStep = hueRange*1.0 / quart;
        currColor = baseColor;
        int pos=0;

        for (int i = 0; i < quart*4; i++) {
          if (i<quart){direction=1;}
          if (i>=quart && i<quart*2){direction=-1;}
          if (i>=quart*2 && i<quart*3){direction=-1;}
          if (i>=quart*3 && i<quart*4){direction=1;}
          
          hue += hueStep*direction;
          currColor.h = (int) hue;
          pos=(loopCount+i) % NUM_LEDS;
          //leds[pos]=currColor;
          hsv2rgb_rainbow( currColor, leds[pos]);
        }
    }
}



void shiftBand(){
  for (int i = 0; i < NUM_LEDS; i++) {
    currColor.h = ( baseColor.h+i+loopCount);
    leds[i]=currColor;
  }
  loopCount++;
}

void sineHue(){

      static uint8_t hue_index = 0;
      static uint8_t led_index = 0;
 
      if (led_index <= 0||led_index >= NUM_LEDS) {
        direction=direction*-1;
      }
      for (int i = 0; i < NUM_LEDS; i = i + 1)
      {
        leds[i] = CHSV(hue_index, 255, 255 - int(abs(sin(float(i + led_index) / NUM_LEDS * 2 * 3.14159) * 255)));
      }

      led_index +=direction;
      hue_index -=direction;

     if (hue_index >= 255) {
        hue_index = 0;
      }
      if (hue_index <= 0) {
        hue_index = 255;
      }    
      delayMultiplier = 2;
         
}

void solid() {
  if (firstRun){
    FastLED.clear();
    fillHsv(baseColor);
    FastLED.show();
    firstRun=false;
  }
}
/////////////////////////////////////////////////////////


CRGB temp2rgb(unsigned int kelvin) {
    int tmp_internal = kelvin / 100.0;
    int red, green, blue;
    CRGB result;
    
    // red 
    if (tmp_internal <= 66) {
        red = 255;
    } else {
        float tmp_red = 329.698727446 * pow(tmp_internal - 60, -0.1332047592);
        if (tmp_red < 0) {
            red = 0;
        } else if (tmp_red > 255) {
            red = 255;
        } else {
            red = tmp_red;
        }
    }
    
    // green
    if (tmp_internal <=66){
        float tmp_green = 99.4708025861 * log(tmp_internal) - 161.1195681661;
        if (tmp_green < 0) {
            green = 0;
        } else if (tmp_green > 255) {
            green = 255;
        } else {
            green = tmp_green;
        }
    } else {
        float tmp_green = 288.1221695283 * pow(tmp_internal - 60, -0.0755148492);
        if (tmp_green < 0) {
            green = 0;
        } else if (tmp_green > 255) {
            green = 255;
        } else {
            green = tmp_green;
        }
    }
    
    // blue
    if (tmp_internal >=66) {
        blue = 255;
    } else if (tmp_internal <= 19) {
        blue = 0;
    } else {
        float tmp_blue = 138.5177312231 * log(tmp_internal - 10) - 305.0447927307;
        if (tmp_blue < 0) {
            blue = 0;
        } else if (tmp_blue > 255) {
            blue = 255;
        } else {
            blue = tmp_blue;
        }
    }
    result = CRGB(red,green,blue);
    return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/********************************** START CALLBACK*****************************************/


void callback(char* topic, byte* payload, unsigned int length) {
  Serial << "callback\n";
    
  StaticJsonDocument<256> root;
  deserializeJson(root, payload, length);


 if (root.containsKey("scanWifi")) {
    if (strcmp(root["scanWifi"], "ON") == 0) {
      scanne = true;
    }
    else if (strcmp(root["scanWifi"], "OFF") == 0) {
      scanne = false;
    }
  }
  
  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      stateOn = true;
      firstRun=true;
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
      setColor(0, 0, 0);
      cycleMode = "none";
    }
  }
  

    if (root.containsKey("hueRange")) {
      hueRange = root["hueRange"];
    }

     if (root.containsKey("quart")) {
      quart = root["quart"];
    }  

     if (root.containsKey("animFactor")) {
      animFactor = root["animFactor"];
    }  


    if (root.containsKey("stepTime")) {
      stepTime = root["stepTime"];
    }

    if (root.containsKey("bright")) {
      brightness = root["bright"];
      FastLED.setBrightness(brightness);
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
      FastLED.setBrightness(brightness);
    }

 
    
   if (root.containsKey("kelvin")) {
      unsigned int kelvin  = root["kelvin"];
      CRGB tempColor = temp2rgb(kelvin);
      effect="solid";
      fill_solid(leds,NUM_LEDS,tempColor);
      FastLED.show();
      firstRun=false;
      cycleMode = "none";
    }
 
    if (root.containsKey("colorHsv")) {
      baseColor.h = root["colorHsv"]["h"];
      baseColor.s = root["colorHsv"]["s"];
      baseColor.v = root["colorHsv"]["v"];
      
      fillHsv(baseColor);
    }

    // setFilenameByName

     if (root.containsKey("showByIndex")){
      int idx = root["showByIndex"];
      getDataByIndex(idx);
    }

    /* ************************************** cycle ************/
    if (root.containsKey("cycleInt")) {
        cycleMode = "cycleInt";
        cycleVon = 0;
        cycleBis = 15;

          int von  = root["cycleInt"]["von"];
          cycleVon = von;
          
          int bis  = root["cycleInt"]["bis"];
          if (bis > 0) {cycleBis = bis;}

          Serial << cycleMode << F(" von: ") << von << F(" bis: ") << bis  << F(" \n ");          
          cycleIndex=von;
      
    }

    if (root.containsKey("cycleExt")) {
        cycleMode = "cycleExt";
        cycleVon = 0;
        cycleBis = findexEntryCnt;

          int von  = root["cycleExt"]["von"];
          cycleVon = von;
          
          int bis  = root["cycleExt"]["bis"];
          if (bis > 0) {cycleBis = bis;}

          Serial << cycleMode << F(" von: ") << von << F(" bis: ") << bis  << F(" \n ");          
          cycleIndex=von;

    }

    if (root.containsKey("showByName")) {
      String fileN = root["showByName"];
      char fn[15]={};
      //char* fn=fileN;
      fileN.toCharArray(fn, 15);
      setFilenameByName(fn);
      readImgData ();
    }

    
   if (root.containsKey("cycleByName")) {
    String data=root["cycleByName"];
    // make sure the last one gets packaged too
    data += ",";
    String dbgLine="";
    
    int maxIndex = data.length() - 1;
    char separator=',';
    int entry = 0;
    int cnt =0;

    for (int i = 0; i <= maxIndex; i++)
    {
        if (data[i] == ' '){
          // eat it
        }
        
        else if (data[i] != separator){
          cycleSteps[entry][cnt]=data[i];
          cnt++;
        }
        else
        {
          cycleSteps[entry][cnt]=0;
          String fn = String(   cycleSteps[entry] );
          debug(fn+" entry: "+String(entry),false); 
          entry++;
          cnt=0; 
        }
    }
    cycleVon=0;
    cycleBis=entry-1;
    cycleIndex=0;
    cycleMode="cycleByName";


//    dbgLine="";
//    for (int i=0;i<21;i++){
//      dbgLine += String(cycleSteps[i]);
//      dbgLine += " ";
//    }
//    debug(dbgLine,false);
    
   }

    /* ******************* effects ***************************/


    if (root.containsKey("effect")) {
      cycleMode = "none";
      firstRun=true;
      effect = root["effect"];
      effectString=effect;
      Serial << effect << "\n";

      if (effect == "solid"){
        fillHsv(baseColor);
      }
      
      if (effectString == "adhoc"){
        Serial << "adhoc Ast\n";
        if (root.containsKey("data")){
          String data=root["data"];
          data += ",";
          //Serial << data << "\n";

          int maxIndex = data.length() - 1;
          String chunkVal = "";    
          char separator=',';
          int j = 0;
 
          for (int i = 0; i <= maxIndex; i++)
          {
              if (data[i] != separator){
                chunkVal.concat(data[i]);
              }
              else
              {
                //Serial <<F("|") << chunkVal<<F("|\n") ;
                 chunkVal.trim();
                 int anz=chunkVal.length();
                 
                 char hex[anz+1];
                 strcpy(hex,chunkVal.c_str());
                 long test = hex2int(hex);
                 
                //Serial <<F("|") << chunkVal<<"| : "<< test << F("|\n") ;

                 if (chunkVal != ""){
                   adhoc[j] = test;

                  // Serial << j << F(": ") << chunkVal<< F("->")  << adhoc[j] << F("\n") ;
                   chunkVal = "";
                   j++;
                   
                 }
              }
                      
        }    
      }
    } 
  }



  if (stateOn) {

    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
  }
  else {

    realRed = 0;
    realGreen = 0;
    realBlue = 0;
  }
 
  sendState();
}

/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */


/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonDocument<BUFFER_SIZE> root;

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  root["bright"] = brightness;
  root["effect"] = effect;
  root["stepTime"] = stepTime;
  root["vers"] = version;
   
//  JsonObject colorHsv = root.createNestedObject("colorHsv");
//  colorHsv["h"] = baseColor.h;
//  colorHsv["s"] = baseColor.s;
//  colorHsv["v"] = baseColor.v;
//
//  JsonObject currColour = root.createNestedObject("currColor");
//  currColour["h"] = currColor.h;
//  currColour["s"] = currColor.s;
//  currColour["v"] = currColor.v;
//  
 
  char buffer[512];
  serializeJson(root, buffer);

  mqClient.publish(state_topic, buffer, true);
}



///////////////////////////////////////////////////////////////////////////////
// send a message to mq
void sendDbg(String msg){
  StaticJsonDocument<BUFFER_SIZE> doc;
 
  doc["dbg"]=msg;
  

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  mqClient.publish(dbg_topic, buffer, n);
}

// called out of timed_loop async
void checkDebug(){
  if (msgCount>0){
    
    String message = arrMessages[0];

     for (int i = 0; i < numMsg-1; i++) {
      arrMessages[i]=arrMessages[i+1];
    }
    arrMessages[numMsg-1]="";
    msgCount--;
    sendDbg(message);
  }
  
  
}

// stuff the line into an array. Another function will send it to mq later
void debug(String dbgMsg, boolean withSerial){
  //Serial << "dbgMsg: " << dbgMsg <<  "\n";
  
  if (withSerial) {
    Serial.println( dbgMsg );
  }
  if (msgCount<numMsg){
    arrMessages[msgCount]=dbgMsg;
    msgCount++;
  }
  
}


void scanWifi(){
  WiFi.disconnect();
  delay(100);

  /*
   * pubsub mag es nicht, die ganze Liste auf einen rutsch zu posten
   * Deshalb mal ein post je netz
   * umzuordnen zu können, scanId und reportId angeben
   */
   
  scanId++;
   
  // WiFi.scanNetworks will return the number of networks found
  int netCnt = WiFi.scanNetworks();
  
  setupWifi();
  delay(50);
  if (!mqClient.connected()) {
        mqConnect();
  }  
  delay(50);    
  debug(String(netCnt)+" SSIDS erkannt",false);
  
  if (netCnt == 0) {
    Serial << F("no networks found\n");
  } else {
    //Serial << (netCnt) << F(" networks found\n");

   
    for (int i = 0; i < netCnt; ++i) {
      StaticJsonDocument<256> doc;
    
      doc["scanner"] = String(SENSORNAME); 
      doc["netCnt"] = netCnt;
      doc["scanId"] = scanId;
    
      JsonArray nets = doc.createNestedArray("nets");

      JsonObject obj = nets.createNestedObject();
      
      obj["bssid"] = WiFi.BSSIDstr(i);
      obj["ssid"] = WiFi.SSID(i);
      obj["rssi"] = WiFi.RSSI(i);
      obj["chnl"] = WiFi.channel(i);
      //obj["isHidden"] = WiFi.isHidden(i);
      
      doc["dtlId"] = i;

      char buffer[256];
      serializeJson(doc, buffer);
      mqClient.publish(scan_topic, buffer);
      
      //Serial << WiFi.SSID(i) << F(" | ") << WiFi.RSSI(i) << F(" | ") << WiFi.BSSIDstr(i) << F(" | ") << WiFi.channel(i) << F(" | ") << WiFi.isHidden(i)  << F("\n");
      
      delay(10);

    }
  }
}



/////////////////////////////////////////////////////////////////

void setupWifi(){
  // Connect to WiFi

  // make sure there is no default AP set up unintended
  WiFi.mode(WIFI_STA);
  //WiFi.hostname(HOSTNAME);
  
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  msg = "WiFi connected, local IP: "+WiFi.localIP().toString();
  debug(msg,true);
  
}


void setupMq(){
  // pubsub setup
  mqClient.setServer(mqtt_server, mqtt_port);
  mqClient.setCallback(callback);
  mqConnect();  
}


void setupOta(){

  //OTA SETUP
  ArduinoOTA.setPort(OTAport);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(SENSORNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    debug("Starting OTA",false);
  });
  ArduinoOTA.onEnd([]() {
    debug("End OTA",false);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debug("OTA Auth Failed",false);
    else if (error == OTA_BEGIN_ERROR) debug("OTA Begin Failed",false);
    else if (error == OTA_CONNECT_ERROR) debug("OTA Connect Failed",false);
    else if (error == OTA_RECEIVE_ERROR) debug("OTA Receive Failed",false);
    else if (error == OTA_END_ERROR) debug("OTA End Failed",false);
  });
  ArduinoOTA.begin();
  
}




/********************************** START mosquitto *****************************************/
/*
 * // Possible values for client.state()
#define MQTT_CONNECTION_TIMEOUT     -4
#define MQTT_CONNECTION_LOST        -3
#define MQTT_CONNECT_FAILED         -2
#define MQTT_DISCONNECTED           -1
#define MQTT_CONNECTED               0
#define MQTT_CONNECT_BAD_PROTOCOL    1
#define MQTT_CONNECT_BAD_CLIENT_ID   2
#define MQTT_CONNECT_UNAVAILABLE     3
#define MQTT_CONNECT_BAD_CREDENTIALS 4
#define MQTT_CONNECT_UNAUTHORIZED 5
 * 
 */
void mqConnect() {
  // Loop until we're reconnected <- komplett unnütz
  //while (!mqClient.connected()) {

    // Attempt to connect
    if (mqClient.connect(SENSORNAME, mqtt_username, mqtt_password)) {
      
      mqClient.subscribe(set_topic);      
    } else {
      Serial.print((WiFi.status() == WL_CONNECTED));
      Serial.print("mqConnect failed, rc=");
      Serial.println(mqClient.state());

      Serial << F("Wifi: ") << WiFi.status() << F(" mqtt_username: ") << mqtt_username << F("\n");
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  //}

}

/////////////////////////////////////////////////////////

void setupSpiff(){

  //Initialize File System
  if(SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }



// dir
//  Dir dir = SPIFFS.openDir("/");
//  // or Dir dir = LittleFS.openDir("/data");
//  while (dir.next()) {
//      Serial.print(dir.fileName());
//      if(dir.fileSize()) {
//          File f = dir.openFile("r");
//          Serial.print(" : ");
//          Serial.println(f.size());
//      }
//  }
//  
//  FSInfo fs_info;
//  SPIFFS.info(fs_info);
//  Serial.println(fs_info.totalBytes);
//  
//  Serial.println(fs_info.usedBytes);
//  Serial.println(fs_info.blockSize);
//  Serial.println(fs_info.pageSize);
//  Serial.println(fs_info.maxOpenFiles);
//  Serial.println(fs_info.maxPathLength);
//  
//  Serial.print("done");  
}
