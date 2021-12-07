#include <Plotter.h>


#define PITCH 1
#define ROLL 2
#define YAW 0
#define ENC_A 2       // пин энкодера
#define ENC_B 7       // пин энкодера
#define ENC_TYPE 1    // тип энкодера, 0 или 1

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include "Wire.h"

// Объявляем задействованные дискретные каналы контроллера для связи
SoftwareSerial outSerial(11, 12); // RX, TX

//----/servo----------------------------------------------------
Servo motorPlus;
Servo motorMinus;
int motor_torque = 55;
//----/end_servo------------------------------------------------
//----/encoder--------------------------------------------------
volatile int32_t encCounter;
volatile boolean state0, lastState, turnFlag;
//----/end_encoder----------------------------------------------

//----/mpu6050--------------------------------------------------
MPU6050 mpu;
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
uint8_t mpuIntStatus, devStatus, fifoBuffer[164];
uint16_t packetSize, fifoCount,gx, gy, gz;
float toAngle = 180 / M_PI;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int rates[3];           // angle rates
//----/end_mpu6050----------------------------------------------

//----/scheduler_logic------------------------------------------

uint8_t mode = 10;
uint32_t tper = 2000;
uint32_t totkl = 30000;
uint32_t time_start = 0;
uint32_t curr_tact = 0;
bool mode_running = false;

//----/end_scheduler_logic--------------------------------------

//----/control--------------------------------------------------

enum controls {
  PLUS,         // 0
  MINUS,        // 1
};

double sigma = 0;
double z = 0;
double x_meas;
float wz = 0.0;
float omega = 0.0;
float xplt = 0.0;
float Fprev = 0;
float F = 0;
double w_zero = 0.0;
int switch_curr = 0;
int switch_prev = 0;

double x01 = 0.0;
double x02 = 0.0;
double k1 = 1.5;
double k2 = 1.5;
double alpha1 = 20;
double alpha2 = 30;
double h1 = 5;
double h2 = 5;

 
double m4_theta = 0; 
double m4_z0 = 0;   
double m4_prev_tick_time= 0;
double m4_curr_tick_time=0;
double m4_k_vos = 0;

float x_offset;

double Tvos = 2.0;  
double k_vos = 150;  

double ext_mom = 255;
//----/end_control------------------------------------------------


void (*OnModeStart)()  = NULL;
bool (*OnModeStep)()  = NULL;
void (*OnModeStop)()  = NULL;


//----/parsrer----------------------------------------------------

char divider = ' ';
char ending = ';';
const char *headers[]  = {
  "mode",    // 0
  "k1vl",    // 1
  "a1vl",    // 2
  "h1vl",    // 3
  "k2vl",    // 4
  "a2vl",    // 5
  "h2vl",    // 6
  "zwco",    // 7
  "motf",    // 8
  "tper",    // 9
  "totk",    // 10
  "x01v",    // 11
  "x02v",    // 12
  "kvos",    // 13
  "tvos",    // 14
  "extm",    // 15
  "stx0",    // 16
};

enum names {
  MODE,          // 0
  K1_VAL,        // 1
  ALPH1_VAL,     // 2
  H1_VAL,        // 3
  K2_VAL,        // 4
  ALPH2_VAL,     // 5
  H2_VAL,        // 6
  ZERO_WZ_COND,  // 7
  MOTOR_F,       // 8
  TPER_VAL,        // 9
  TOTKL_VAL,        // 10
  X01_VAL,        // 11
  X02_VAL,        // 12
  KVS_VAL,        // 13
  TVS_VAL,        // 14
  EMOM_VAL,        // 15
  SET_X0,         // 16
};
names thisName;
byte headers_am = sizeof(headers) / 2;
uint32_t prsTimer;
String prsValue = "";
String prsHeader = "";
enum stages {WAIT, HEADER, GOT_HEADER, VALUE, SUCCESS};
stages parseStage = WAIT;
boolean recievedFlag;

//----/end_parser------------------------------------------



class middle3 {
  public:
    float middleOf3(float value);  // возвращает фильтрованное значение
  private:
    float _buf[3];
    byte _counter = 0;
};

float middle3::middleOf3(float value) {
  _buf[_counter] = value;
  if (++_counter > 2) _counter = 0;
  float _middle;

  if ((_buf[0] <= _buf[1]) && (_buf[0] <= _buf[2])) {
    _middle = (_buf[1] <= _buf[2]) ? _buf[1] : _buf[2];
  } else {
    if ((_buf[1] <= _buf[0]) && (_buf[1] <= _buf[2])) {
      _middle = (_buf[0] <= _buf[2]) ? _buf[0] : _buf[2];
    } else {
      _middle = (_buf[0] <= _buf[1]) ? _buf[0] : _buf[1];
    }
  }
  return _middle;
}

middle3 filX, filY, filZ;

void setup() {
  
  outSerial.begin(9600);
  Wire.begin();
  attachInterrupt(0, int0, CHANGE);
  Serial.begin(9600);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  analogWrite(5, 255);
  analogWrite(6, 255);

  TWBR = 24;
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  motorPlus.attach(9);
  motorMinus.attach(10);  
  motorPlus.write(0);
  motorMinus.write(0);
 
  delay(5000);
  
}

double sign(double val)
{  
    return (val >= 0) ? 1 : -1;  
}

void GetMeasures()
{  
  GetAngles();
  x_meas = float(encCounter)*360.0/2000.0;
  xplt = x_meas;
  wz = filZ.middleOf3(mpuRoll);
}

void calc_F(float k, float h, float alpha, float x0, float x_sub = 0.0)
{
  float x = x_meas + x0 - x_offset;
  if(x_sub != 0.0)
    x = x_sub;
  sigma = x+k*(float)wz;
  F = 0.5*(sign(sigma - alpha + h*Fprev)+sign(sigma + alpha + h*Fprev));
}

void loop() {
  curr_tact = millis();
  commandsReadProcess();
  
  if(!mode_running)  
    return;
  else{    
    while(millis() - curr_tact < 10)
    ;
  }
  ModeStep();
}

void ChangeMode(int mode)
{  
   switch (mode) {
      case 10: 
        mode_running = false;
        return;
      case 0: 
      {
        OnModeStart = Mode0Start;
        OnModeStep = Mode0Step;
        OnModeStop = Mode0Stop;
      }
        break;
      case 1: 
      {
        OnModeStart = Mode1Start;
        OnModeStep = Mode1Step;
        OnModeStop = Mode1Stop;        
      }
        break;
      case 2:
      {
        OnModeStart = Mode2Start;
        OnModeStep = Mode2Step;
        OnModeStop = Mode2Stop; 
      }
        break;      
      case 4: 
      {        
        OnModeStart = Mode4Start;
        OnModeStep = Mode4Step;
        OnModeStop = Mode4Stop; 
      }
        break;
      default: 
        return;
    }
    
    mode_running = true;
    OnModeStart();
}

void ModeStep()
{ 
  if(OnModeStep())
  {
    mode_running = false;
    OnModeStop();
  }
  Serial.print("grph ");Serial.print(xplt);Serial.print(" ");Serial.print(wz);Serial.print(" ");Serial.print(F);Serial.print(" ");Serial.print(millis() - time_start);Serial.print(" ");Serial.print(sigma);Serial.println();   
}

void int0() {
  state0 = bitRead(PIND, ENC_A);
  if (state0 != lastState) {
#if (ENC_TYPE == 1)
    turnFlag = !turnFlag;
    if (turnFlag) {
      if (bitRead(PIND, ENC_B) != lastState) encCounter++;
      else encCounter--;
    }
#else
    if (bitRead(PIND, ENC_B) != lastState) encCounter++;
    else encCounter--;
#endif    
    lastState = state0;
  }
}

void ApplyControl(int8_t control, int32_t val)
{
  if(control == MINUS){
     motorMinus.write(val);
     motorPlus.write(0);
  }
  else{
    motorMinus.write(0);
    motorPlus.write(val);
  }
    
}

void RelayFcn()
{
  if(F>0.5)
  {
    ApplyControl(MINUS, motor_torque);
    switch_curr = -1;
    if(switch_curr != switch_prev)
    {
        switch_prev = switch_curr;
        Fprev = F;
    }
  }
  else
  {
    if(F<-0.5)
    {
      ApplyControl(PLUS, motor_torque);
      switch_curr = 1;
      if(switch_curr != switch_prev)
      {
          switch_prev = switch_curr;
          Fprev = F;
      }
    }
    else
    {
       switch_curr = 0;
      if(switch_curr != switch_prev)
      {
          switch_prev = switch_curr;
          Fprev = F;
      }
       ApplyControl(MINUS, 0);
    }
  }
}

void GetAngles() {
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize)
    return;
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.resetFIFO();
  mpu.dmpGetGyro(rates,fifoBuffer);
  mpuPitch = rates[PITCH] ;
  mpuRoll = rates[ROLL] ;
  mpuYaw  = rates[YAW] ;
  mpu.resetFIFO();
}

void parsingSeparate() {
  if (outSerial.available() > 0) {
    if (parseStage == WAIT) {
      parseStage = HEADER;
      prsHeader = "";
      prsValue = "";
    }
    if (parseStage == GOT_HEADER)
      parseStage = VALUE;
    char incoming = (char)outSerial.read();
    if (incoming == divider) {
      parseStage = GOT_HEADER;
    }
    else if (incoming == ending) {
      parseStage = SUCCESS;
    }
    if (parseStage == HEADER)
      prsHeader += incoming;
    else if (parseStage == VALUE)
      prsValue += incoming;
    prsTimer = millis();
  }
  if (parseStage == SUCCESS) {
    for (byte i = 0; i < headers_am; i++) 
    { 
      if (prsHeader == headers[i]) 
        thisName = i;  
    } 
    recievedFlag = true; 
    parseStage = WAIT; 
  } if ((millis() - prsTimer > 10) && (parseStage != WAIT))      
      parseStage = WAIT; 
}

void commandsReadProcess()
{  
  parsingSeparate();
  if (recievedFlag) {
    recievedFlag = false;
    switch (thisName) {
      case MODE: mode = prsValue.toInt();  delay(5000); ChangeMode(mode); time_start = millis();
        break;
      case K1_VAL: k1 = prsValue.toFloat(); //Serial.print("k1 set to: "); Serial.println(k1);
        break;
      case ALPH1_VAL: alpha1 = prsValue.toFloat();// Serial.print("alpha1 set to: "); Serial.println(alpha1);
        break;
      case H1_VAL: h1 = prsValue.toFloat();// Serial.print("h1 set to: "); Serial.println(h1);
        break;
      case K2_VAL: k2 = prsValue.toFloat();// Serial.print("k2 set to: "); Serial.println(k2);
        break;
      case ALPH2_VAL: alpha2 = prsValue.toFloat();// Serial.print("alpha2 set to: "); Serial.println(alpha2);
        break;
      case H2_VAL: h2 = prsValue.toFloat(); //Serial.print("h2 set to: "); Serial.println(h2);
        break;
      case TPER_VAL: tper = uint32_t(1000*prsValue.toFloat()); //Serial.print("tper set to: "); Serial.println(tper);
        break;
      case TOTKL_VAL: totkl = uint32_t(1000*prsValue.toFloat()); //Serial.print("totkl set to: "); Serial.println(totkl);
        break;
      case ZERO_WZ_COND: w_zero = prsValue.toFloat();// Serial.print("initial_w_z set to: "); Serial.println(w_zero);
        break;
      case MOTOR_F: motor_torque = prsValue.toInt(); //Serial.print("motor_torque set to: "); Serial.println(motor_torque);
        break;
      case X01_VAL: x01 = prsValue.toFloat(); //Serial.print("x01 set to: "); Serial.println(x01);
        break;
      case X02_VAL: x02 = prsValue.toFloat();// Serial.print("x02 set to: "); Serial.println(x02);
        break;
      case KVS_VAL: k_vos = prsValue.toFloat(); //Serial.print("k_vos set to: "); Serial.println(k_vos);
        break;
      case TVS_VAL: Tvos = prsValue.toFloat(); //Serial.print("Tvos set to: "); Serial.println(Tvos);
        break;
      case EMOM_VAL: ext_mom = prsValue.toInt(); //Serial.print("ext_mom set to: "); Serial.println(ext_mom);
        break;
      case SET_X0: encCounter = int32_t(prsValue.toFloat()/360.0*2000.0); 
        break;
    }
  }
}

void initialAccelStart()
{
  delay(15);
}

bool initialAccelStep()
{  
  GetMeasures(); 
  if(w_zero < 0.0)
  {
    if(wz > w_zero + 0.1*(8.18*58-370)){             //поправка на задержку, иначе просто wz > w_zero
      ApplyControl(MINUS, motor_torque);
      return false;
    }
    else
      return true;      
     
  } else {
    if(wz < w_zero - 0.07*(8.18*58-370)){     //поправка на задержку, иначе просто wz > w_zero    
      ApplyControl(PLUS, motor_torque);
      return false;
    }
    else
      return true;   
  }     
}

void initialAccelStop()
{
    ApplyControl(MINUS, 0);
    x_offset = xplt;
}


void Mode0Start()
{
    initialAccelStart();
}

bool Mode0Step()
{
  return initialAccelStep();
}

void Mode0Stop()
{
  delay(10);
  mode = 1;
  ChangeMode(mode);
}

void Mode1Start()
{
    analogWrite(5, ext_mom);       // - внешнее возмущение
    time_start = millis();
    delay(10);
}

bool Mode1Step()
{
  if(millis() - time_start <  tper)
  {
    analogWrite(5, ext_mom); 
    GetMeasures();  
    calc_F(k1, h1, alpha1, x01);
    RelayFcn(); 
    return false;
  }
  return true;  
}

void Mode1Stop()
{
  delay(10);
  mode = 2;
  analogWrite(5, 255); 
  ChangeMode(mode);
}

void Mode2Start()
{
  delay(10);
  analogWrite(5, ext_mom);
}

bool Mode2Step()
{ 
  if(millis() - time_start <  (totkl - tper))
  {
    analogWrite(5, ext_mom); 
    GetMeasures();  
    calc_F(k2, h2, alpha2, x02);
    RelayFcn();
    return false;
  }     
  return true;  
}

void Mode2Stop()
{
  delay(10);
  ApplyControl(MINUS, 0);
  analogWrite(5, 255); 
  mode = 10;
  ChangeMode(mode);
}

void Mode4Start()
{
  while(initialAccelStep())
    delay(10);
  initialAccelStop();
  analogWrite(5, ext_mom);  
  time_start = millis();
   
  m4_theta = 0; 
  m4_z0 = 0;   
  m4_prev_tick_time= millis();
  m4_curr_tick_time= millis();
  m4_k_vos = k_vos;
}

void Mode4Step()
{        
  if(millis() - time_start < totkl)
  {
    GetMeasures(); 
    
    m4_curr_tick_time= millis();       //текущий момент времени t(i)    
    m4_theta+= (m4_curr_tick_time - m4_prev_tick_time)/1000.0;   //добавляем к времени импульса (t(i) -  t(i-1))/1000 (мc -> с)    
    m4_prev_tick_time = m4_curr_tick_time;  // запомиаем момент t(i), для следующего такта это будет t(i-1)    
    double z = m4_z0*exp(-m4_theta/Tvos) + m4_k_vos*(1 - exp(-m4_theta/Tvos));
    double sgm = x_meas - z;
    
    calc_F(0.0, h1, alpha1, 0.0, sgm);
         
    if(F != Fprev)
    {
      m4_k_vos = k_vos*F;  //меняем знак экспоненты в ос    
      m4_z0 = z;
      m4_theta = 0;
    }    
    RelayFcn();
    sigma = z;
  }  
}

void Mode4Stop()
{
  delay(10);
  analogWrite(5, 255);
  ApplyControl(MINUS, 0);
  mode = 10;
  ChangeMode(mode);
}



