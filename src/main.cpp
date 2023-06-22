#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include "at24c256.h"
// SET_LOOP_TASK_STACK_SIZE( 16*1024 );
at24c256 eep(0x50); // 0x50 = A1(GND), A0(GND)

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define NUM_DISPLAY 3

#if NUM_DISPLAY == 2
#define MAX_DEVICES 16
#define NUM_ZONE 2
#elif NUM_DISPLAY == 3
#define MAX_DEVICES 24
#define NUM_ZONE 3
#else
#define MAX_DEVICES 8
#define NUM_ZONE 1
#endif

#define DATA_PIN 23 // or MOSI
#define CS_PIN 5    // or SS
#define CLK_PIN 18  // or SCK
MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
const uint8_t FONT_ONE[] = {5, 0, 4, 2, 127, 0};
const uint8_t FONT_SPACE[] = {5, 0, 0, 0, 0, 0};
const uint8_t FONT_MINUS[] = {5, 8, 8, 8, 8, 8};
const uint8_t FONT_EQUAL[] = {5, 20, 20, 20, 20, 20};
const uint8_t FONT_ONEBLANK[] = {1, 0};
const uint8_t FONT_TWOBLANK[] = {2, 0, 0};

#define RUN_LED_pin 2

//=== Serial COM ================================================
enum
{
  COM_RX_NULL = 0,
  COM_RX_OK,
  COM_RX_NG,
  COM_RX_WRONG_MODE
};

enum
{
  NOT_NUMBER = 0,
  VALUE_RANGE_OK,
  VALUE_RANGE_OUT
};
#define ASCII_CR 13
#define ASCII_LF 10
#define TERMINATION 0x0d // return

//=== Serial COM 0 ================================================
#define MAX_RX_BUFFER 50
char rx[MAX_RX_BUFFER];
byte rx_index = 0;
byte rx_status = 0;
char tx0[MAX_RX_BUFFER];

//=== Serial COM 2 ================================================
#define RXD2 16
#define TXD2 17

//=== Commands ====================================================
#define CMD_BUF 30
#define CMD_SIZE 3
char command[CMD_SIZE][CMD_BUF];
//===============================================================
#define MAX_SEQ_NUM 10
#define MAX_TIM_NUM MAX_SEQ_NUM
#define US MAX_TIM_NUM
enum
{
  T00 = 0,
  T01,
  T02,
  T03,
  T04,
  T05,
  T06,
  T07,
  T08,
  T09,
  T10,
  T11,
  T12,
  T13,
  T14,
  T15,
  T16,
  T17,
  T18,
  T19,
  U00,
  U01,
  U02,
  U03,
  U04,
  U05,
  U06,
  U07,
  U08,
  U09,
  U10,
  U11,
  U12,
  U13,
  U14,
  U15,
  U16,
  U17,
  U18,
  U19
};
enum
{
  TIMOUT = 0,
  TIMSET
};
unsigned long time_data[(MAX_TIM_NUM + US)], timeset[(MAX_TIM_NUM + US)];
byte t_status[(MAX_TIM_NUM + US)];
int seq[MAX_SEQ_NUM];

uint16_t spacing, brightness;
bool auto_mode;
char text0[CMD_BUF];
char text1[CMD_BUF];
#define EEP_SPACING 0x00
#define EEP_BRIGHTNESS 0x02
#define EEP_AUTO_MODE 0x04
#define EEP_TEXT_0 0x10
#define EEP_TEXT_1 0x50

#if NUM_DISPLAY == 3
char text2[CMD_BUF];
#define EEP_TEXT_2 0x90
#endif
// FUNCTION DECLARATION========================================
void seq0();
void seq1();
void seq2();
void seq3();
void seq4();
template <class T>
int eep_update(int ee, const T &value);
template <class T>
int eep_read(int ee, T &value);
void display(byte z, char s[]);
int toInt(char cstr[]);
bool isEqual(char arr[], String s);
void print_help();
void print_the_date_and_file_of_sketch(void);
byte rx_command(char _rx[], byte _serial_num);
void serialEvent2();
void serialEvent();
void rx_clear(void);
byte set_timer(byte temp_tim_num, unsigned long temp_tim_set);
byte timer_status(byte temp_tim_num);
void init_io(void);
//===================================================================
void setup()
{
  byte i;
  for (i = 0; i < (MAX_TIM_NUM + US); i++)
  {
    t_status[i] = TIMOUT;
  }
  for (i = 0; i < MAX_SEQ_NUM; i++)
  {
    seq[i] = 0;
  }
  eep.init();
  eep_read(EEP_SPACING, spacing);
  if (spacing > 15)
    spacing = 0;
  eep_read(EEP_BRIGHTNESS, brightness);
  if (brightness > 15)
    brightness = 10;
  eep_read(EEP_AUTO_MODE, auto_mode);
  eep_read(EEP_TEXT_0, text0);
  if (text0[0] == 255)
    text0[0] = 0;
  eep_read(EEP_TEXT_1, text1);
  if (text1[0] == 255)
    text1[0] = 0;
#if NUM_DISPLAY == 3
  eep_read(EEP_TEXT_2, text2);
  if (text2[0] == 255)
    text2[0] = 0;
#endif

  delay(500);
  init_io();
  Serial.begin(115200);
  Serial.println();
  P.setIntensity(brightness);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("Program Begin..."));
  print_the_date_and_file_of_sketch();
  serialEvent();
  serialEvent2();
  seq[1] = 10;
  Serial.print("Current mode: ");
  Serial.println(auto_mode ? "Auto mode" : "Manual mode");
  Serial.println("To read mode: @read mode");
  Serial.println("To change mode: @mode x | x = auto, only can receive commands from master.");
  Serial.println("                        | x = manual, can work as standalone.");
  
  Serial2.print("@getslave\r");
} // end setup

void loop()
{
  seq0(); // Heart Beat
  seq1(); //
  seq2(); //
  seq3(); //
  seq4();
}

void seq0(void)
{
  switch (seq[0])
  {
  case 0:
    if (timer_status(T00) == TIMOUT)
    {
      set_timer(T00, 500);
      digitalWrite(RUN_LED_pin, HIGH);
      seq[0] = 10;
    }
    break;
  case 10:
    if (timer_status(T00) == TIMOUT)
    {
      set_timer(T00, 500);
      digitalWrite(RUN_LED_pin, LOW);
      seq[0] = 0;
    }
    break;
  default:
    break;
  }
}

void seq1(void)
{
  switch (seq[1])
  {
  case 0:

    break;
  case 10:

    break;
  case 20:

    break;
  default:
    break;
  }
}

void seq2(void)
{
  switch (seq[2])
  {
  case 0:

    break;
  case 10:
    break;
  case 20:
    break;
  default:
    break;
  }
}

void seq3(void)
{
  switch (seq[3])
  {
  case 0:
    break;
  case 10:
    break;
  case 20:
    break;
  default:
    break;
  }
}

void seq4(void)
{
  switch (seq[4])
  {
  case 0:
    break;
  case 10:
    break;
  case 20:
    break;
  default:
    break;
  }
}

//== Custom  Function ============================================================
template <class T>
int eep_update(int ee, const T &value)
{
  T reads;
  eep_read(ee, reads);
  if (value == reads)
    return -1;

  const byte *p = (const byte *)(const void *)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    eep.write(ee++, *p++);
  return i;
}

template <class T>
int eep_read(int ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = eep.read(ee++);
  return i;
}

void display(byte z, char s[])
{
  char *text = s;
  if (z >= NUM_DISPLAY || s[0] == 0)
    return;
  if (z == 0 || z == 2)
    P.displayZoneText(z, text, PA_RIGHT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  else if (z == 1)
    P.displayZoneText(z, text, PA_LEFT, 0, 0, PA_PRINT, PA_NO_EFFECT);
  if (P.displayAnimate())
  {
    if (P.getZoneStatus(z))
    {
      P.displayReset(z);
    }
  }
}

int toInt(char cstr[])
{
  const int sz = strlen(cstr);
  long res = 0;
  if (sz >= 6)
    return 65535;
  for (int i = 0; i < sz; i++)
  {
    if (cstr[i] < '0' || cstr[i] > '9')
      return 65535;
    res += (cstr[i] - '0') * pow(10, sz - i - 1);
  }
  if (res > 65535 || res < -65535)
    return 65535;
  return res;
}

bool isEqual(char arr[], String s)
{
  return strcmp(arr, s.c_str()) == 0;
}

void print_help()
{
  Serial.println(F("===================================================================================="));
  Serial.println(F("|Commands          | Description                                                   |"));
  Serial.println(F("|@help             | Menu of commands                                              |"));

#if NUM_DISPLAY == 3
  Serial.println(F("|@text x [TEXT]    | x = xth display (0-2)                                         |"));
#elif NUM_DISPLAY == 2
  Serial.println(F("|@text x [TEXT]    | x = xth display (0-1)                                         |"));
#endif
  Serial.println(F("|                  | TEXT = text to display                                        |"));
  Serial.println(F("|@spacing x        | x = any number to set spacing between characters              |"));
  Serial.println(F("|@brightness x     | x = brightness (0-15)                                         |"));
  Serial.println(F("|@read mode        | Read current mode                                             |"));
  Serial.println(F("|@mode x           | x = auto, is auto mode, only can receive commands from master |"));
  Serial.println(F("|                  | x = manual, is manual mode, can work as standalone            |"));
  Serial.println(F("|                                                                                  |"));
  Serial.println(F("| FOR CUSTOMIZE DISPLAY : '|' = one space, '~' = two space                         |"));
  Serial.println(F("===================================================================================="));
}
//==========
void print_the_date_and_file_of_sketch(void)
{
  // Print compile time and file name to serial monitor
  Serial.print("Compiled on ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);

  // Print sketch file name without directory to serial monitor
  Serial.print("Sketch file: ");
  String fileName = __FILE__;
  char *baseName = strrchr(fileName.c_str(), '\\');
  if (baseName != NULL)
  {
    baseName++;
    Serial.println(baseName);
  }
  else
  {
    Serial.println(fileName);
  }
}

//== Serial COMM 0  Function ============================================================
// UART0

byte rx_command(char _rx[], byte _serial_num)
{
  for (int i = 0; i < CMD_SIZE; i++)
    command[i][0] = 0; // reset command array
  String in_str = _rx;
  byte sz = in_str.length();
  byte _rx_status = COM_RX_NULL;
  bool in_msg = 0;
  byte start_idx = 0;
  byte cmd_idx = 0;
  bool skip_next = 0;
  for (int i = 0; i < sz; i++)
  {
    if (skip_next)
    {
      skip_next = 0;
      continue;
    }
    if (in_str[i] == '[')
    {
      in_msg = 1;
      start_idx++;
    }
    if (in_str[i] == ']')
    {
      in_msg = 0;
      in_str.substring(start_idx, i).toCharArray((command[cmd_idx]), CMD_BUF);
      skip_next = 1;
    }
    else if ((in_str[i] == ' ' && !in_msg) || i == sz - 1)
    {
      if (cmd_idx >= CMD_SIZE)
      {
        Serial.println("COMMAND TOO MUCH ARGUMENTS");
        break;
      }
      if (i - start_idx >= CMD_BUF - 1)
        Serial.println("COMMAND TOO LONG");
      in_str.substring(start_idx, i).toCharArray((command[cmd_idx]), CMD_BUF);
      cmd_idx++;
      start_idx = i + 1;
    }
  }
  _rx_status = COM_RX_NG;
  if (auto_mode && _serial_num == 0)
  {
    if ((isEqual(command[0], "@mode") || isEqual(command[0], "@read")) == 0)
      return COM_RX_WRONG_MODE;
  }
  else if (auto_mode == 0 && _serial_num == 2)
    return COM_RX_WRONG_MODE;
  if (isEqual(command[0], "@help"))
  {
    print_help();
    _rx_status = COM_RX_OK;
  }
  else if (isEqual(command[0], "@text"))
  {
    byte _zone = toInt(command[1]);
    if (_zone < NUM_ZONE)
    {
      if (_zone == 0)
        eep_update(EEP_TEXT_0, command[2]);
      else if (_zone == 1)
        eep_update(EEP_TEXT_1, command[2]);
#if NUM_DISPLAY == 3
      else if (_zone == 2)
        eep_update(EEP_TEXT_2, command[2]);
#endif
      display(_zone, command[2]);
      _rx_status = COM_RX_OK;
    }
    else
      Serial.println("Please enter the correct zone");
  }
  else if (isEqual(command[0], "@spacing"))
  {
    spacing = toInt(command[1]);
    P.setCharSpacing(spacing);
    eep_update(EEP_SPACING, spacing);
    Serial.println("Character spacing has been set to " + String(spacing));
    Serial.println("New spacing will be updated in the next message");
    _rx_status = COM_RX_OK;
  }
  else if (isEqual(command[0], "@brightness"))
  {
    brightness = toInt(command[1]);
    P.setIntensity(brightness);
    eep_update(EEP_BRIGHTNESS, brightness);
    Serial.println("Display brightness has been set to " + String(brightness));
    _rx_status = COM_RX_OK;
  }
  else if (isEqual(command[0], "@mode"))
  {
    if (isEqual(command[1], "auto"))
    {
      auto_mode = true;
      eep_update(EEP_AUTO_MODE, auto_mode);
      brightness = 5;
      eep_update(EEP_BRIGHTNESS, brightness);
      spacing = 0;
      eep_update(EEP_SPACING, spacing);
      char BLANK[] = "         ";
      display(0, BLANK);
      display(1, BLANK);
      display(2, BLANK);
      eep_update(EEP_TEXT_0, "");
      eep_update(EEP_TEXT_1, "");
      eep_update(EEP_TEXT_2, "");
      Serial.println("Auto mode, only can receive commands from master.");
      _rx_status = COM_RX_OK;
    }
    else if (isEqual(command[1], "manual"))
    {
      auto_mode = false;
      eep_update(EEP_AUTO_MODE, auto_mode);
      Serial.println("Manual mode, can work as standalone.");
      _rx_status = COM_RX_OK;
    }
  }
  else if (isEqual(command[0], "@read"))
  {
    if (isEqual(command[1], "mode"))
    {
      Serial.print("Current mode: ");
      Serial.println(auto_mode ? "Auto" : "Manual");
      _rx_status = COM_RX_OK;
    }
  }

  return _rx_status;
}

void serialEvent2()
{
  char _inChar = 0;
  rx_index = 0;
  while (Serial2.available())
  {
    _inChar = (char)Serial2.read();
    rx[rx_index] = _inChar;
    if (rx_index <= MAX_RX_BUFFER)
      rx_index++;
    else
      rx_index = 0;
    if (_inChar == ASCII_CR)
    {
      rx_status = rx_command(rx, 2);
      if (rx_status == COM_RX_OK)
      {
        rx[0] = '*';
        Serial.println(rx);
      }
      else if (rx_status == COM_RX_NULL)
      {
      }
      else if (rx_status == COM_RX_WRONG_MODE)
      {
        Serial.println("*WRONG MODE, This is manual mode which can only read commands from pc");
        Serial.println("If you wish to use this display, turn on auto mode by using this command -> @mode auto");
      }
      else
      {
        Serial.println("*CMD_ERROR");
      }
      rx_index = 0;
      rx_clear();
    }
  }
}
void serialEvent()
{
  char _inChar = 0;
  rx_index = 0;
  while (Serial.available())
  {
    _inChar = (char)Serial.read();
    // Serial.print(_inChar);
    rx[rx_index] = _inChar;
    if (rx_index <= MAX_RX_BUFFER)
      rx_index++;
    else
      rx_index = 0;
    if (_inChar == ASCII_CR)
    {
      rx_status = rx_command(rx, 0);
      if (rx_status == COM_RX_OK)
      {
        rx[0] = '*';
        Serial.println(rx);
      }
      else if (rx_status == COM_RX_NULL)
      {
      }
      else if (rx_status == COM_RX_WRONG_MODE)
      {
        Serial.println("*WRONG MODE, This is auto mode which can only read commands from master");
        Serial.println("If you wish to use this display, turn on manual mode by using this command -> @mode manual");
      }
      else
        Serial.println("*CMD_ERROR");
      rx_index = 0;
      rx_clear();
    }
  }
}

void rx_clear(void)
{
  for (int i = 0; i < MAX_RX_BUFFER; i++)
    rx[i] = 0;
}

//== Timer Function ===============================================================
byte set_timer(byte temp_tim_num, unsigned long temp_tim_set)
{
  if (temp_tim_num < MAX_TIM_NUM)
  {
    time_data[temp_tim_num] = millis();
    timeset[temp_tim_num] = temp_tim_set;
    t_status[temp_tim_num] = TIMSET;
    return 1;
  }
  else if (temp_tim_num >= MAX_TIM_NUM && temp_tim_num < (MAX_TIM_NUM + US))
  {
    time_data[temp_tim_num] = micros();
    timeset[temp_tim_num] = temp_tim_set;
    t_status[temp_tim_num] = TIMSET;
    return 1;
  }
  else
  {
    return 0;
  }
}

byte timer_status(byte temp_tim_num)
{
  byte temp_return = TIMOUT;
  if (temp_tim_num < MAX_TIM_NUM)
  {
    if (t_status[temp_tim_num] == TIMSET)
    {
      if ((millis() - time_data[temp_tim_num]) >= timeset[temp_tim_num])
        t_status[temp_tim_num] = TIMOUT;
      temp_return = t_status[temp_tim_num];
    }
  }
  else if (temp_tim_num >= MAX_TIM_NUM && temp_tim_num < (MAX_TIM_NUM + US))
  {
    if (t_status[temp_tim_num] == TIMSET)
    {
      if ((micros() - time_data[temp_tim_num]) >= timeset[temp_tim_num])
        t_status[temp_tim_num] = TIMOUT;
      temp_return = t_status[temp_tim_num];
    }
  }
  return temp_return;
}

//====== Initialize IO =====================================================
void init_io(void)
{
  pinMode(RUN_LED_pin, OUTPUT);
  digitalWrite(RUN_LED_pin, LOW); // Output, nomal LOW
  P.begin(NUM_ZONE);
  delay(1000);
  P.addChar('1', FONT_ONE);
  P.addChar(' ', FONT_SPACE);
  P.addChar('=', FONT_EQUAL);
  P.addChar('-', FONT_MINUS);
  P.addChar('|', FONT_ONEBLANK);
  P.addChar('~', FONT_TWOBLANK);
  P.setCharSpacing(spacing);
  P.displayClear();
#if NUM_DISPLAY == 2
  P.setZone(0, 0, 7);
  P.setZone(1, 8, 15);
  P.setZoneEffect(0, 1, PA_FLIP_UD);
  P.setZoneEffect(0, 1, PA_FLIP_LR);
  display(0, text0);
  display(1, text1);
#elif NUM_DISPLAY == 3
  P.setZone(0, 0, 7);
  P.setZone(1, 8, 15);
  P.setZone(2, 16, 23);
  P.setZoneEffect(0, 1, PA_FLIP_UD);
  P.setZoneEffect(0, 1, PA_FLIP_LR);
  P.setZoneEffect(2, 1, PA_FLIP_UD);
  P.setZoneEffect(2, 1, PA_FLIP_LR);
  delay(250);
  display(0, text0);
  delay(250);
  display(1, text1);
  delay(250);
  display(2, text2);
  delay(250);
#else
  P.setZone(0, 0, 7);
  P.setZoneEffect(0, 1, PA_FLIP_UD);
  P.setZoneEffect(0, 1, PA_FLIP_LR);
#endif
}
