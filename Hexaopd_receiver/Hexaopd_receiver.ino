#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define USMIN 600   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2500  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50
#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal
#define g_led 7
#define b_led 6
#define y_led 5
#define o_led 4
#define r_led 3

float rp_const = 5;
float v1, v2;      // roll,pitch 0->2
int b = rp_const;  // equation of plane : b*v1*x + b*v2*y + b^2*z = 0 => z = (v1*x + v2*y)/b; find this z for each leg, and then add absolute_z_rest to it
float Z_projection;

int flag = 1, newButtonVal, oldButtonVal = 1, newButtonVal2, oldButtonVal2, flag2 = 1, mode = 1;
int i = 0;
const uint64_t pipeIn = 0xF9E8F0F0E2LL;
unsigned long lastRecvTime = 0, newTime = 0;

float coxa_length = 7.54;
float femur_length = 10.0;
float tibia_length = 15.2;
float C_angle;
float F_angle;
float T_angle;
const double Y_Rest = 7.54 + 9.0;
const double Z_Rest = 0;
int car_mode = 0;
double absolute_z_rest;
double absolute_y_rest;

RF24 radio(9, 8);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);


int new_on_off_value = 1, old_on_off_value = 1, on_off_state = -1;
int next_button_value;
int gait_index_val = 0;
double max_step_height, max_step_length, default_roll, default_pitch = 0, x_coordinate, y_coordinate, prev_x_coordinate = 0, prev_y_coordinate = 0, body_height = 7.2, body_height_up, body_height_down;
double new_body_height_time, old_body_height_time;
String gaits[] = { "tri_gait", "wave", "CAR", "adaptive" };  //tri_gait / wave_gait / quad_gait / adaptive_gait
String current_gait = "tri_gate";
double t = 0, n = 0.02;  // t is the current timestep, and n is the number of divisions

int dt = 0;  // time interval between successive iterations of step trajectory.

struct PacketData {
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
};
PacketData receiverData;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {  // p
  double x;       // O
  double y;       // I
  double z;       // N
} Point3D;

Point3D start = { 0.0, 0.0, 0.0 }, control = { 0.0, 0.0, 0.0 }, end = { 0.0, 0.0, 0.0 };
// T
Point3D quadraticBezier3D(Point3D start, Point3D control, Point3D end, double t) {  // S
  Point3D point;
  double one_minus_t = 1.0 - t;

  // Calculate the point on the curve using the formula
  point.x = one_minus_t * one_minus_t * start.x + 2 * one_minus_t * t * control.x + t * t * end.x;
  point.y = one_minus_t * one_minus_t * start.y + 2 * one_minus_t * t * control.y + t * t * end.y;
  point.z = one_minus_t * one_minus_t * start.z + 2 * one_minus_t * t * control.z + t * t * end.z;
  return point;
}
Point3D linearBezier3D(Point3D start, Point3D end, double t) {
  Point3D point;

  // Calculate the point on the curve using the linear interpolation formula
  point.x = start.x + t * (end.x - start.x);
  point.y = start.y + t * (end.y - start.y);
  point.z = start.z + t * (end.z - start.z);

  return point;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double modify(byte x) {
  double return_val;
  double x_2 = (double)x;
  return_val = x_2 * max_step_length / 255.0 - (max_step_length / 2);
  return return_val;
}
void project_x_and_y_on_circle() {
  if (hypot(x_coordinate, y_coordinate) > max_step_length / 2) {
    float radius = max_step_length / 2;
    float constant = sqrt((x_coordinate * x_coordinate) / (y_coordinate * y_coordinate));
    if (y_coordinate >= 0)
      y_coordinate = radius / sqrt(1 + (x_coordinate * x_coordinate) / (y_coordinate * y_coordinate));
    else
      y_coordinate = -1 * radius / sqrt(1 + (x_coordinate * x_coordinate) / (y_coordinate * y_coordinate));
    if (x_coordinate >= 0)
      x_coordinate = constant * abs(y_coordinate);
    else
      x_coordinate = -1 * constant * (y_coordinate);
  }
}

void set_variable_values() {
  max_step_height = map(receiverData.max_step_height, 0, 255, 0.0, 15);
  max_step_length = map(receiverData.max_step_length, 0, 255, 0, 10);
  default_roll = ((float)receiverData.default_roll) * 2.0 / 255.0 - 1;
  default_pitch = ((float)receiverData.default_pitch) * 2.0 / 255.0 - 1;
  v1 = default_roll;
  v2 = default_pitch;
  x_coordinate = 0.9 * prev_x_coordinate + 0.1 * modify(receiverData.y_coordinate) - 0.004;
  prev_x_coordinate = x_coordinate;
  y_coordinate = 0.9 * prev_y_coordinate + -0.1 * modify(receiverData.x_coordinate) - 0.007;
  prev_y_coordinate = y_coordinate;
  project_x_and_y_on_circle();
  body_height_up = receiverData.default_body_height_up;
  body_height_down = receiverData.default_body_height_down;
  new_on_off_value = receiverData.on_off;
  current_gait = gaits[receiverData.gait];
  if (receiverData.gait == 2) {
    car_mode = 1;
  } else {
    car_mode = 0;
  }
}

void set_variable_default_values() {
  on_off_state = -1;
}

void execute_main_code() {
  update_on_off_state();
  if (on_off_state == -1) {
    analogWrite(r_led, 0);
    set_variable_default_values();
  } else if (on_off_state == 1) {
    analogWrite(r_led, 100);
    update_body_height();
    generate_3_points();
    execute_gait(current_gait);
  }
  Serial.print(" step_height:");  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.print(max_step_height);
  Serial.print(" step_length:");
  Serial.print(max_step_length);
  Serial.print(" roll:");
  Serial.print(default_roll);
  Serial.print(" pitch:");
  Serial.print(default_pitch);
  Serial.print(" x:");
  Serial.print(x_coordinate);
  Serial.print(" y:");
  Serial.print(y_coordinate);      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.print(" current_gait:");  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(current_gait);

  //   generate_3_points();
  //   execute_gait(current_gait);
}

void generate_3_points() {
  start = { x_coordinate, y_coordinate, 0.0 };
  control = { 0, 0, max_step_height * hypot(x_coordinate, y_coordinate) / (max_step_length / 2) };
  end = { -x_coordinate, -y_coordinate, 0.0 };
}

void execute_gait(String gait) {
  if (gait == "tri_gait") {
    // Serial.print(" execuying tri");
    execute_tri_gait();
  } else if (gait == "wave") {
    // Serial.print(" execuying wave");
    execute_wave_gait();
  } else if (gait == "CAR") {
    // Serial.print(" execuying CAR");
    execute_car_gait();
  } else {
    // Serial.print(" execuying adaptive");
    execute_adaptive_gait();
  }
  t = t + n;
  if (t > 2.0)
    t = 0;
  delay(dt);
}

void execute_tri_gait() {
  digitalWrite(g_led, HIGH);
  digitalWrite(b_led, LOW);
  digitalWrite(y_led, LOW);
  digitalWrite(o_led, LOW);

  move_leg(t + 0.0 / 2.0, 1);
  move_leg(t + 2.0 / 2.0, 2);
  move_leg(t + 0.0 / 2.0, 3);
  move_leg(t + 2.0 / 2.0, 4);
  move_leg(t + 0.0 / 2.0, 5);
  move_leg(t + 2.0 / 2.0, 6);
}
void execute_wave_gait() {
  digitalWrite(b_led, HIGH);
  digitalWrite(g_led, LOW);
  digitalWrite(y_led, LOW);
  digitalWrite(o_led, LOW);

  move_leg(t + 0.0 / 6.0, 1);
  move_leg(t + 2.0 / 6.0, 2);
  move_leg(t + 4.0 / 6.0, 3);
  move_leg(t + 6.0 / 6.0, 6);
  move_leg(t + 8.0 / 6.0, 5);
  move_leg(t + 10.0 / 6.0, 4);
}
void execute_car_gait() {
  digitalWrite(y_led, HIGH);
  digitalWrite(g_led, LOW);
  digitalWrite(b_led, LOW);
  digitalWrite(o_led, LOW);

  move_car(t + 0.0 / 2.0, 1);
  move_car(t + 2.0 / 2.0, 2);
  move_car(t + 0.0 / 2.0, 3);
  move_car(t + 2.0 / 2.0, 4);
  move_car(t + 0.0 / 2.0, 5);
  move_car(t + 2.0 / 2.0, 6);

  // move_leg(t+0.0/2.0 , 1);
  // move_leg(t+2.0/2.0 , 2);
  // move_leg(t+0.0/2.0 , 3);
  // move_leg(t+2.0/2.0 , 4);
  // move_leg(t+0.0/2.0 , 5);
  // move_leg(t+2.0/2.0 , 6);
}
void move_car(float time_step, int leg_no) {
  if (time_step > 2)
    time_step = time_step - 2.0;
  Point3D point;


  if (receiverData.y_coordinate > 200 || receiverData.y_coordinate < 50) {
    if (receiverData.x_coordinate < 200 && receiverData.x_coordinate > 50) {
      if (time_step <= 1.0) {
        point = linearBezier3D(start, end, time_step);
      } else if (time_step <= 2.0) {
        point = quadraticBezier3D(end, control, start, time_step - 1.0);
      }
    } else {
      point = { 0, 0, 0 };
    }

  } else if (receiverData.y_coordinate > 50) {
    if (receiverData.x_coordinate > 200 || receiverData.x_coordinate < 50) {
      if (leg_no > 3) {
        if (time_step <= 1.0) {
          point = linearBezier3D(start, end, time_step);
        } else if (time_step <= 2.0) {
          point = quadraticBezier3D(end, control, start, time_step - 1.0);
        }
      } else {
        if (time_step <= 1.0) {
          point = linearBezier3D(end, start, time_step);
        } else if (time_step <= 2.0) {
          point = quadraticBezier3D(start, control, end, time_step - 1.0);
        }
      }
    } else {
      point = { 0, 0, 0 };
    }
  }


  // if(leg_no > 3){
  //     if (time_step <= 1.0) {
  //         point = linearBezier3D(start, end, time_step);
  //     }
  //     else if (time_step <= 2.0) {
  //         point = quadraticBezier3D(end, control, start, time_step - 1.0);
  //     }
  // }
  // else{
  //     if (time_step <= 1.0) {
  //         point = linearBezier3D(end, start, time_step);
  //     }
  //     else if (time_step <= 2.0) {
  //         point = quadraticBezier3D(start, control, end, time_step - 1.0);
  //     }
  // }
  CartesianMove(point.x, point.y, point.z, leg_no);
}
void execute_adaptive_gait() {
  digitalWrite(o_led, HIGH);
  digitalWrite(g_led, LOW);
  digitalWrite(y_led, LOW);
  digitalWrite(b_led, LOW);

  // move_leg(t+0.0/2.0 , 1);
  // move_leg(t+2.0/2.0 , 2);
  // move_leg(t+0.0/2.0 , 3);
  // move_leg(t+2.0/2.0 , 4);
  // move_leg(t+0.0/2.0 , 5);
  // move_leg(t+2.0/2.0 , 6);
}
void move_leg(float time_step, int leg_no) {
  if (time_step > 2)
    time_step = time_step - 2.0;
  Point3D point;
  double theta;
  switch (leg_no) {
    case 1: theta = 2.0944; break;
    case 2: theta = 3.1415; break;
    case 3: theta = 4.1888; break;
    case 4: theta = 1.0472; break;
    case 5: theta = 0.0000; break;
    case 6: theta = -1.0472; break;
    default: Serial.print("ERROR IN LEG_NO");
  }
  Point3D temp_start, temp_end;
  temp_start.x = start.x * cos(theta) - start.y * sin(theta);
  temp_start.y = start.x * sin(theta) + start.y * cos(theta);
  temp_start.z = start.z;
  temp_end.z = end.z;
  temp_end.x = -temp_start.x;
  temp_end.y = -temp_start.y;
  if (time_step <= 1.0) {
    point = linearBezier3D(temp_start, temp_end, time_step);
  } else if (time_step <= 2.0) {
    point = quadraticBezier3D(temp_end, control, temp_start, time_step - 1.0);
  }
  // Serial.print(temp_start.x);
  // Serial.print(" ");
  // Serial.print(temp_start.y);
  // Serial.print(" ");
  // Serial.print(temp_end.x);
  // Serial.print(" ");
  // Serial.print(temp_end.y);
  // Serial.print(" ");
  // Serial.print(1);
  // Serial.print("    ");
  // Serial.print("time_step: ");
  // Serial.print(time_step);
  CartesianMove(point.x, point.y, point.z, leg_no);
}

void CartesianMove(double X, double Y, double Z, int servo_no) {

  Y += Y_Rest;
  absolute_y_rest = Y;
  // Y += Y_Rest + 5;
  Z += Z_Rest - body_height;
  absolute_z_rest = Z;

  if (current_gait == "tri_gait" || current_gait == "wave") {
    Z_projection = project_z_on_plane(servo_no) + absolute_z_rest;
  } else {
    Z_projection = absolute_z_rest;
  }

  // Serial.print("absolute y_rest z_rest : ");
  // Serial.print( absolute_y_rest);
  Serial.print(" leg_no: ");
  Serial.print(servo_no);
  Serial.print(" absolute_z_rest: ");
  Serial.print(absolute_z_rest);
  Serial.print(" z projection : ");
  Serial.print(Z_projection);
  // Z = -10;
  float J1L = coxa_length;
  float J2L = femur_length;
  float J3L = tibia_length;
  // CALCULATE INVERSE KINEMATIC SOLUTION
  double J1 = atan(X / Y) * (180 / PI);
  double H = sqrt((Y * Y) + (X * X));
  double L = sqrt(((H - J1L) * (H - J1L)) + (Z_projection * Z_projection));
  double J3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
  double A = atan(Z_projection / (H - J1L)) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  double J2 = (B + A);                                     // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'
  UpdatePosition(J1, J2, J3, servo_no);
}

float project_z_on_plane(int servo_no) {
  Point3D point;
  float pl = 15 + absolute_y_rest;
  float z_projection;

  switch (servo_no) {
    case 1:
      point = { pl * cos(1.047), pl * sin(1.047), 0 };
      break;
    case 2:
      point = { pl, 0, 0 };
      break;
    case 3:
      point = { pl * cos(1.047), -pl * sin(1.047), 0 };
      break;
    case 4:
      point = { -pl * cos(1.047), pl * sin(1.047), 0 };
      break;
    case 5:
      point = { -pl, 0, 0 };
      break;
    case 6:
      point = { -pl * cos(1.047), -pl * sin(1.047), 0 };
      break;
    default:
      Serial.print(" error in project z on plane");
  }

  z_projection = (v1 * point.x + v2 * point.y) / b;
  return (z_projection);
}

void UpdatePosition(double J1, double J2, double J3, int servo_no) {
  // MOVE TO POSITION
  float angleUS0 = map(90 - J1, 0, 180, USMIN, USMAX);
  float angleUS1 = map(90 - J2, 0, 180, USMIN, USMAX);
  float angleUS2 = map(J3, 0, 180, USMIN, USMAX);

  if (car_mode == 0) {
    switch (servo_no) {
      case 1:
        pwm2.writeMicroseconds(15, angleUS0);
        pwm2.writeMicroseconds(14, angleUS1);
        pwm2.writeMicroseconds(13, angleUS2);
        break;
      case 2:
        pwm2.writeMicroseconds(11, angleUS0);
        pwm2.writeMicroseconds(10, angleUS1);
        pwm2.writeMicroseconds(9, angleUS2);
        break;
      case 3:
        pwm2.writeMicroseconds(7, angleUS0);
        pwm2.writeMicroseconds(6, angleUS1);
        pwm2.writeMicroseconds(5, angleUS2);
        break;
      case 4:
        pwm.writeMicroseconds(0, angleUS0);
        pwm.writeMicroseconds(1, angleUS1);
        pwm.writeMicroseconds(2, angleUS2);
        break;
      case 5:
        pwm.writeMicroseconds(5, angleUS0);
        pwm.writeMicroseconds(6, angleUS1);
        pwm.writeMicroseconds(7, angleUS2);
        break;
      case 6:
        pwm.writeMicroseconds(8, angleUS0);
        pwm.writeMicroseconds(9, angleUS1);
        pwm.writeMicroseconds(10, angleUS2);
        break;
      default: Serial.print(" error in update_position");
    }
  } else {
    switch (servo_no) {
      case 1:
        pwm2.writeMicroseconds(15, angleUS0 - 633);
        pwm2.writeMicroseconds(14, angleUS1);
        pwm2.writeMicroseconds(13, angleUS2);
        break;
      case 2:
        pwm2.writeMicroseconds(11, angleUS0);
        pwm2.writeMicroseconds(10, angleUS1);
        pwm2.writeMicroseconds(9, angleUS2);
        break;
      case 3:
        pwm2.writeMicroseconds(7, angleUS0 + 633);
        pwm2.writeMicroseconds(6, angleUS1);
        pwm2.writeMicroseconds(5, angleUS2);
        break;
      case 4:
        pwm.writeMicroseconds(0, angleUS0 + 633);
        pwm.writeMicroseconds(1, angleUS1);
        pwm.writeMicroseconds(2, angleUS2);
        break;
      case 5:
        pwm.writeMicroseconds(5, angleUS0);
        pwm.writeMicroseconds(6, angleUS1);
        pwm.writeMicroseconds(7, angleUS2);
        break;
      case 6:
        pwm.writeMicroseconds(8, angleUS0 - 633);
        pwm.writeMicroseconds(9, angleUS1);
        pwm.writeMicroseconds(10, angleUS2);
        break;
      default: Serial.print(" error in update_position");
    }
  }
  // Serial.print(" mode CAR = ");
  // Serial.print(car_mode);
}



void update_on_off_state() {
  new_on_off_value = receiverData.on_off;
  if (new_on_off_value == 0 && old_on_off_value == 1) {
    on_off_state *= -1;
  }
  old_on_off_value = new_on_off_value;
  Serial.print(" on/off_state:");
  Serial.print(on_off_state);
}
void update_body_height() {
  float absolute_max_body_height = 15.20;
  float absolute_min_body_height = 0.00;

  new_body_height_time = millis();
  if (new_body_height_time - old_body_height_time > 100) {
    body_height += (1 - body_height_up) / 10;
    body_height -= (1 - body_height_down) / 10;
    old_body_height_time = new_body_height_time;
    if (body_height > absolute_max_body_height)
      body_height = absolute_max_body_height;
    else if (body_height < absolute_min_body_height)
      body_height = absolute_min_body_height;
  }
  Serial.print(" body_height:");  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.print(body_height);      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void collapse() {
  int ca = 800, fa = 600, ta = 800;
  pwm2.writeMicroseconds(15, ca);
  pwm2.writeMicroseconds(14, fa);
  pwm2.writeMicroseconds(13, ta);
  pwm2.writeMicroseconds(11, ca);
  pwm2.writeMicroseconds(10, fa);
  pwm2.writeMicroseconds(9, ta);
  pwm2.writeMicroseconds(7, ca);
  pwm2.writeMicroseconds(6, fa);
  pwm2.writeMicroseconds(5, ta);
  pwm.writeMicroseconds(0, ca);
  pwm.writeMicroseconds(1, fa);
  pwm.writeMicroseconds(2, ta);
  pwm.writeMicroseconds(5, ca);
  pwm.writeMicroseconds(6, fa);
  pwm.writeMicroseconds(7, ta);
  pwm.writeMicroseconds(8, ca);
  pwm.writeMicroseconds(9, fa);
  pwm.writeMicroseconds(10, ta);
}

void setup() {
  pinMode(g_led, OUTPUT);
  pinMode(b_led, OUTPUT);
  pinMode(y_led, OUTPUT);
  pinMode(o_led, OUTPUT);
  pinMode(r_led, OUTPUT);
  Serial.begin(1000000);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();  //start the radio receiver
  set_variable_default_values();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  pwm2.begin();
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);
  delay(10);
}

void loop() {
  // Check if RF is connected and packet is available
  if (radio.isChipConnected() && radio.available()) {
    radio.read(&receiverData, sizeof(PacketData));
    lastRecvTime = millis();
    set_variable_values();
  } else {
    //Check Signal lost.
    unsigned long now = millis();
    if (now - lastRecvTime > SIGNAL_TIMEOUT) {
      set_variable_default_values();
      collapse();
    }
  }


  execute_main_code();

  ///////////////////////////////////////////////////////////////////////////////////
}