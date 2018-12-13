#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(4, 3);

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

int izqA = 6; 
int izqB = 5; 
int derA = 9; 
int derB = 10; 
int vel = 255;            // Velocidad de los motores (0-255)
int bluetoothState = '5';         // inicia detenido

const byte pinObstacle = 7;
const byte arduinoLed = 13;

int haveObstacle = HIGH;
bool isReverseActive = false;

void setup()  { 
  pinMode(derA, OUTPUT);
  pinMode(derB, OUTPUT);
  pinMode(izqA, OUTPUT);
  pinMode(izqB, OUTPUT);

  pinMode(arduinoLed, OUTPUT);

  Serial.begin(9600);
  
  /*Serial.println("Latitude  Longitude   Course");
  Serial.println(" (deg)     (deg) --- to London ----");
  Serial.println("-----------------------------------");
  
  ss.begin(9600);*/
} 
 
void loop()  { 

  if(Serial.available() > 0){        // lee el bluetooth y almacena en bluetoothState
    bluetoothState = Serial.read();
    Serial.println(bluetoothState);
  }
  if(bluetoothState == '1'){           // Boton desplazar al Frente

    if(!canMoveDrone()){
      stopEngines();
      return;
    }
    
    moveForward();
  }

  if(bluetoothState == '2'){          // Boton Reversa
    moveBackward();
  }
  
  if(bluetoothState == '3'){          // Boton IZQ 
    if(!canMoveDrone()){
      stopEngines();
      return;
    }
    
    leftForward();
  }

  if(bluetoothState == '4'){          // Boton DER
    if(!canMoveDrone()){
      stopEngines();
      return;
    }
    
    rightForward();
  }
  
  if(bluetoothState == '5'){         // Boton Parar
    stopEngines();
  } 
}

/* Proximity Methods */

bool canMoveDrone(){
  haveObstacle = digitalRead(pinObstacle);

  if(haveObstacle == LOW){
    digitalWrite(arduinoLed, HIGH);
    return false;
  }

  digitalWrite(arduinoLed, LOW);
  return true;
}

/*Car Methods*/

void moveForward(){
  analogWrite(derB, 0);     
  analogWrite(izqB, 0); 
  analogWrite(derA, vel);
  analogWrite(izqA, vel);
}

void moveBackward(){
  analogWrite(derA, 0);
  analogWrite(izqA, 0);
  analogWrite(derB, vel);
  analogWrite(izqB, vel);
}

void stopForwardEngines(){
  analogWrite(derA, 0);
  analogWrite(izqA, 0);  
}

void stopBackwardEngines(){
  analogWrite(derB, 0);
  analogWrite(izqB, 0);   
}

void stopEngines(){
  stopForwardEngines();
  stopBackwardEngines();
}

void rightForward(){
  analogWrite(derB, 0);
  analogWrite(izqA, 0);
  analogWrite(izqB, vel); 
  analogWrite(derA, vel);
}

void leftForward(){
  analogWrite(derA, 0);
  analogWrite(derB, vel);
  analogWrite(izqB, 0);
  analogWrite(izqA, vel);
}

void rightBackward(){
  analogWrite(derA, 0);
  analogWrite(derB, 0);
  analogWrite(izqA, 0);
  analogWrite(izqB, vel); 
}

void leftBackward(){
  analogWrite(derA, 0);
  analogWrite(izqB, 0);
  analogWrite(izqA, 0);
  analogWrite(derB, vel); 
  
}

/* GPS Methods */

void getGPSPosition(){
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  static const double BRASIL_LAT = -9.602966, BRASIL_LON = -52.613936;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, BRASIL_LAT, BRASIL_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, BRASIL_LAT, BRASIL_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, BRASIL_LAT, BRASIL_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
  
  smartdelay(1000);  
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}
