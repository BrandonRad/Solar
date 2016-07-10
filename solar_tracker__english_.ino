
#include <Servo.h> // include Servo library
#include <DateTime.h> // doc here: http://playground.arduino.cc/Code/DateTime

#define TIME_MSG_LEN  11   // time sync to PC is HEADER and unix time_t as ten ascii digits
#define TIME_HEADER  255   // Header tag for serial time sync message

Servo horizontal;  // horizontal servo
int servoh = 90;   // stand horizontal servo

Servo vertical;   // vertical servo 
int servov = 90;  // stand vertical servo

// LDR pin connections
// name  = analogpin;
int ldrlt = 0; //LDR top left
int ldrrt = 1; //LDR top rigt
int ldrld = 2; //LDR down left
int ldrrd = 3; //ldr down rigt

// Hours active, starts at 07:00, ends at 18:00
int StartHour = 7;
int EndHour = 18;

// Ends 
int MinIntervals = 30;
bool activeOverride = false;
bool active = false;
int lastIntervalTime;

void setup()
{
  Serial.begin(9600);
  // servo connections
  // name.attacht(pin);
  horizontal.attach(9); 
  vertical.attach(10);

  getPCtime();
  Active = true;
  lastIntervalTime = getCurrentMinutes();
}

void loop() 
{ 
  // Sync PC time if possible
  getPCtime();

  if (activeOverride || !checkHoursActive() || !checkInterval())
  {
    SetDefaultServoPosition();
    break;
  }

  // Only run on interval time
  
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt
  
  int dtime = analogRead(4)/20; // read potentiometers  
  int tol = analogRead(5)/4;
  
  int avt = (lt + rt) / 2; // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (lt + ld) / 2; // average value left
  int avr = (rt + rd) / 2; // average value right

  int dvert = avt - avd; // check the diffrence of up and down
  int dhoriz = avl - avr;// check the diffrence og left and rigt

  // check if the diffrence is in the tolerance else change vertical angle
  if (-1*tol > dvert || dvert > tol) 
  {
    if (avt > avd)
    {
      servov = ++servov;
      if (servov > 180) 
      { 
        servov = 180;
      }
    }
    else if (avt < avd)
    {
      servov= --servov;
      if (servov < 0)
      {
        servov = 0;
      }
    }
    vertical.write(servov);
  }

  // check if the diffrence is in the tolerance else change horizontal angle
  if (-1*tol > dhoriz || dhoriz > tol)
  {
    if (avl > avr)
    {
      servoh = --servoh;
      if (servoh < 0)
      {
        servoh = 0;
      }
    }
    else if (avl < avr)
    {
      servoh = ++servoh;
      if (servoh > 180)
      {
        servoh = 180;
      }
    }
    else if (avl = avr)
    {
      // nothing
    }
    horizontal.write(servoh);
  }
  delay(dtime);
}

void SetDefaultServoPosition()
{
  horizontal.write(90);  // horizontal servo
  vertical.write(90);   // vertical servo 
}

int getCurrentMinutes()
{
  return DateTime.Minutes + (DateTime.Hours * 60);
}

bool checkHoursActive()
{
  if (DateTime.Hours >= StartHour && DateTime.Hours <= EndHour)
    return true;
  return false;
}

bool checkInterval()
{
  if (MinInterval > 0)
  {
    if (getCurrentMinutes() - lastIntervalTime >= intervalTime)
    {
      // Switch on or off
      Active = !Active;
      lastIntervalTime = getCurrentMinutes();
    }
  }
}

void getPCtime() {
  // if time available from serial port, sync the DateTime library
  while(Serial.available() >=  TIME_MSG_LEN ){  // time message
    if( Serial.read() == TIME_HEADER ) {        
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){   
        char c= Serial.read();          
        if( c >= '0' && c <= '9')   
          pctime = (10 * pctime) + (c - '0') ; // convert digits to a number            
      }   
      DateTime.sync(pctime);   // Sync DateTime clock to the time received on the serial port
    }  
  }
}

