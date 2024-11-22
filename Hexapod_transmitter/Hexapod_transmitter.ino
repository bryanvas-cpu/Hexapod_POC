#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

const uint64_t pipeOut = 0xF9E8F0F0E2LL;   //IMPORTANT: The same as in the receiver 0xF9E8F0F0E1LL
int r_p = A0, b_p = A1, g_p = A2, y_p = A3, h_p = A6, v_p = A7;
int o_l = 3, y_l = 4, b_l = 9, g_l = 10;
int b_1 = 1, b_2 = 2, b_3 = 0, b_4 = 6, b_5 = 5;

int new_next_val,old_next_val;
int gait_index_val=0,select_val=0;
String gaits[] = {"tri_gait","wave","CAR","adaptive"}; 
String current_gait = "tri_gate";

RF24 radio(8, 7); // select CE,CSN pin
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct PacketData 
{
  byte max_step_height;
  byte max_step_length;
  byte default_roll;
  byte default_pitch;
  byte x_coordinate;
  byte y_coordinate;
  byte default_body_height_up;
  byte default_body_height_down;
  byte on_off;
  byte gait;
  // byte select;
  // byte next;
};
PacketData data;

void select_gait(){
  new_next_val = digitalRead(b_5);
  select_val = digitalRead(b_4);
  if(new_next_val == 0 && old_next_val == 1)
  {
    gait_index_val++;
    if(gait_index_val > 3){
      gait_index_val = 0;
    }
  }
  old_next_val = new_next_val;
  if(select_val == 0){
    current_gait = gaits[gait_index_val];
    data.gait=gait_index_val;
  }
  //display gate value at index specified
  // Serial.print(" current_gait:");//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial.print(current_gait);
  lcd.setCursor(0,0);
  lcd.print("Change: ");
  lcd.print(gaits[gait_index_val]);
  lcd.setCursor(0,1);
  lcd.print("Current: ");
  lcd.print(current_gait);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void mapAndWriteValues()
{

}

void setup()
{
  // Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.stopListening(); //start the radio Transmitter 

  pinMode(b_1,INPUT_PULLUP);
  pinMode(b_2,INPUT_PULLUP);
  pinMode(b_3,INPUT_PULLUP);
  pinMode(b_4,INPUT_PULLUP);
  pinMode(b_5,INPUT_PULLUP);

  lcd.begin();
  lcd.backlight();

}

void loop()
{
  data.max_step_height = map(analogRead(y_p),1023,0,0,255);
  data.max_step_length = map(analogRead(g_p),1023,0,0,255); 
  data.default_roll = map(analogRead(b_p),1023,0,0,255); 
  data.default_pitch = map(analogRead(r_p),1023,0,0,255); 
  data.x_coordinate = map(analogRead(h_p),0,1023,0,255); 
  data.y_coordinate = map(analogRead(v_p),0,1023,0,255); 
  data.default_body_height_up = digitalRead(b_1);
  data.default_body_height_down = digitalRead(b_3);
  data.on_off = digitalRead(b_2);
  select_gait();
  // data.select = digitalRead(b_4);
  // data.next = digitalRead(b_5);

  // Serial.print(data.max_step_height);
  // Serial.print(" ");
  // Serial.print(data.max_step_length);
  // Serial.print(" ");
  // Serial.print(data.default_roll);
  // Serial.print(" ");
  // Serial.print(data.default_pitch);
  // Serial.print(" ");
  // Serial.print(data.x_coordinate);
  // Serial.print(" ");
  // Serial.print(data.y_coordinate);
  // Serial.print(" ");
  // Serial.print(data.default_body_height_up);
  // Serial.print(" ");
  // Serial.print(data.default_body_height_down);
  // Serial.print(" ");
  // Serial.print(data.on_off);
  // Serial.print(" ");
  // Serial.print(data.select);
  // Serial.print(" ");
  // Serial.print(data.next);
  // Serial.println(" ");

  
  

  radio.write(&data, sizeof(PacketData));

}