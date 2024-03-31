#include <Arduino.h>

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <ezButton.h>

//I2C(screen) define

#define SCL 41
#define SDA 42

/*
#define U8X8_PIN_I2C_CLOCK 41
#define U8X8_PIN_I2C_DATA 42
*/

//menu define

#define MENU_SIZE 3

bool menuState = 0;
int menuSelect = 0;
const char *menuType[MENU_SIZE] = {"Piano","Game","About"};
int prevMenuState = 0;

//ADC and buzzer define

#define ADC_PIN 1
#define BUZ_PIN 21

int buzTone;
int prevBuzTone;
int freq = 2000;
int channel = 0;
int resolution = 8;

//rotary encoder define & code

#define CLK_PIN 36  // ESP32 pin GPIO36 connected to the rotary encoder's CLK pin
#define DT_PIN 37   // ESP32 pin GPIO37 connected to the rotary encoder's DT pin
#define SW_PIN 35   // ESP32 pin GPIO35 connected to the rotary encoder's SW pin


volatile int counter = 0;
volatile int direction = 0;
volatile bool encRotated = 0;

void IRAM_ATTR ISR_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(CLK_PIN)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(DT_PIN)) old_AB |= 0x01; // Add current state of pin B  
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    counter++;              // Increase counter
    encval = 0;
    if(menuState == 1)encRotated = 1;
    direction = 1;
  }
  else if( encval < -3 ) {  // Four steps backwards
    counter--;               // Decrease counter
    encval = 0;
    if(menuState == 1)encRotated = 1;
    direction = 0;
  }
}

//ezButton define

ezButton button(SW_PIN);

//screen define & code

#define FONT u8g2_font_inb16_mf
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /*clock=*/SCL, /*data=*/SDA, /*reset=*/U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE, /*clock=*/SCL, /*data=*/SDA);

void setup() {
  // put your setup code here, to run once:


  Serial.begin(115200);
  delay(500);

  //rotary encoder
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  //pinMode(SW_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), ISR_encoder, CHANGE);


  //u8g2 screen
  
  u8g2.begin();
  u8g2.enableUTF8Print();

  //ADC & Buzzer
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZ_PIN, channel);

  button.setDebounceTime(20);
}


void loop() {
  // put your main code here, to run repeatedly:

  //rotary encoder serial print
  static int lastCounter = 0;
  if(counter != lastCounter){
    Serial.println(counter);
    Serial.println(menuState);
    Serial.println(menuType[menuSelect]);

    lastCounter = counter;
  }


  //button
  button.loop();

  if(button.isPressed())
  {
    menuState = !menuState;
  }

  
  //Main menu

  if (menuState == 1)
  {

    u8g2.setFont(u8g2_font_inb16_mf);

    u8g2.firstPage();
    do {
      for (int i = 0; i < MENU_SIZE; i++) 
      {
        u8g2.drawStr(25,i * 22 + 18,menuType[i]);
      }
      if(encRotated)
      {
        encRotated = !encRotated;
        if (direction == 1) // up
        { 
          menuSelect++;
          if (menuSelect > (MENU_SIZE - 1)) menuSelect = 0;
        }
        else // down
        { 
          menuSelect--;
          if (menuSelect < 0) menuSelect = (MENU_SIZE - 1);
        }  
      }
      

      // show cursor at new line:
      u8g2.setCursor(0,menuSelect * 22 + 18);
      u8g2.print('>');
    } while(u8g2.nextPage());

  }
  
  //Piano menu

  if(menuSelect == 0 && menuState == 0)
  {

    if(prevMenuState == 1)counter = 0;

    //static int pianoTone[] = {1,3,5,6,8,10,12,13,15,17,18};
    static int pianoTone[] = {1,3,4,6,8,9,11,13,15,16,18};
    buzTone = (analogRead(ADC_PIN) + 100) / 400;//piano button select
    double freq_p;//precise freqency
    static int freq;

    freq_p = pow(pow(2.0, 1.0 / 12),counter * 12 - 1 + pianoTone[10 - buzTone]) * 440;

  /*
    Serial.print("freq_p =");
    Serial.print(freq_p);
    Serial.print("\n"); 
    Serial.print("buzTone =");
    Serial.print(buzTone);
    Serial.print("\n"); 
  */
    
    if(buzTone != 0) freq = (int)freq_p;
    else freq = 0;

    if(buzTone != prevBuzTone)
      if(buzTone == 0)
      {
        ledcWrite(channel, 0);
      }
      else 
      {
        ledcWrite(channel, 127);
        ledcWriteTone(channel, freq);
      }
    prevBuzTone = buzTone;

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
      u8g2.setCursor(10,20);
      u8g2.print("计数：");
      u8g2.setFont(u8g2_font_inb16_mf);
      u8g2.setCursor(55,22);
      u8g2.print(counter);

      u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
      u8g2.setCursor(10,55);
      u8g2.print("频率：");
      u8g2.setFont(u8g2_font_inb16_mf);
      u8g2.setCursor(55,57);
      u8g2.print(freq);
    } while (u8g2.nextPage());  
  }

  //Game menu

  if(menuSelect == 1 && menuState == 0)
  {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_wqy14_t_gb2312b);
      u8g2.setCursor(10,20);
      u8g2.print("Currently no");
      u8g2.setCursor(10,40);
      u8g2.print("games yet!");

    } while (u8g2.nextPage());  
  }

  //About menu

  if(menuSelect == 2 && menuState == 0)
  {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_wqy14_t_gb2312b);
      u8g2.setCursor(10,20);
      u8g2.print("made for fun");
      u8g2.setCursor(10,40);
      u8g2.print("zssunknown.top");
    } while (u8g2.nextPage());  
  }

  prevMenuState = menuState;

}

// put function definitions here:
