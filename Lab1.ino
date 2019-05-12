// Load library for WIFI
#include <SoftwareSerial.h>
// Load library for LCD display
#include <LiquidCrystal_PCF8574.h>
// Load librarues fir BMP
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
// Load library DHT Humidity & Temperature Sensor  
#include <DHT.h>

#define SSID "Pixel"     // "SSID-WiFiname" 
#define PASS "123456789"       // "password"
#define IP "184.106.153.149"// thingspeak.com ip
String msg = "GET /update?key=PIGU600LAG50NJ3T"; //change it with your api key like "GET /update?key=Your Api Key"

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_PCF8574 lcd(0x27); 
 // Barometric Pressure Sensor
Adafruit_BMP280 bmp;
// WIFI
SoftwareSerial esp8266(9,10); 

// Volatile Variables, used in the interrupt service routine!
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduino finds a beat.

// Regards Serial OutPut  -- Set This Up to your needs
static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse 
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

int onSwitchState = 0;
int offSwitchState = 0;
int dust_pin = 8;
int pulse_pin = 2;
int blink_pin = 13;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000; 
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
int wait_time_ms = 2000;
#define DHTPIN 7
#define DHTTYPE DHT22
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Barometer
float pressure;
float altitude;
// Dust
float concentration;
// Light
float light;
// Temperature
float humidity;
float temp;
// UV
float vsig;
// Pulse
volatile float BPM;

void setup()
{
  // Set serial
  Serial.begin(115200);

  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  esp8266.begin(115200);
  esp8266.println("AT");

  if(esp8266.find("OK")){
    connectWiFi();
    Serial.println("Connect to Internet");
  } else{
    Serial.println("Unable to connect to Internet");
  }
  
  lcd.setBacklight(255);
  lcd.begin(16, 2);
  lcd.print("     Lab 1:     ");
  lcd.setCursor(0,1);
  lcd.print("      Yong      ");
  delay(wait_time_ms);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, INPUT);
  pinMode(5, INPUT);
  interruptSetup(); 
}

void loop()
{
  run_barometer();
  check_button();
  run_dust();
  check_button();
  run_light();
  check_button();
  run_temperature();
  check_button();
  run_uv();
  check_button();
  run_pulse();
  check_button();
  push_data();
  check_button();
  check_alert();
  check_button();
}

void check_button()
{
  onSwitchState = digitalRead(2);
  offSwitchState = digitalRead(5);
  if (offSwitchState == HIGH)
  {
    lcd.clear();
    lcd.print("Program on hold");
    Serial.println("Off button pressed");
    delay(wait_time_ms);
  }
  if (onSwitchState == HIGH)
  {
    lcd.clear();
    lcd.print("Program on");
    Serial.println("On button pressed");
    delay(wait_time_ms);
  }
}

void check_alert()
{
  if (light > 200)
  {
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    lcd.clear();
    lcd.print("[Warning]");
    lcd.setCursor(0,1);
    lcd.print("Too bright.");
    delay(wait_time_ms);
  }
  else
  {
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    lcd.clear();
    lcd.print("[Clear]");
    lcd.setCursor(0,1);
    lcd.print("Safe.");
    delay(wait_time_ms);
  }
}

void run_barometer()
{
  // Begin Check
  lcd.clear();
  lcd.print("[Barometer]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);

  // Run Check
  pressure = bmp.readPressure();
  Serial.println("Pressure = " + String(pressure) + " Pa");
  lcd.clear();
  lcd.print("Pressure:");
  lcd.setCursor(0, 1);
  lcd.print(String(pressure) + " Pa");
  delay(wait_time_ms);
  altitude = bmp.readAltitude(1013.25);
  Serial.println("Altitude = " + String(altitude) + " m");
  lcd.clear();
  lcd.print("Altitude:");
  lcd.setCursor(0, 1);
  lcd.print(String(altitude) + " m");
  delay(wait_time_ms);
  
  // End check
  lcd.clear();
  lcd.print("[Barometer]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void run_dust()
{
  // Begin Check
  lcd.clear();
  lcd.print("[Dust]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);
  
  // Run Check
  duration = pulseIn(dust_pin, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  if ((millis() - starttime) >= sampletime_ms)
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);  
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; 
    Serial.println("Concentration = " + String(concentration) + " pcs/0.01cf");
    
    lcd.clear();
    lcd.print("Concentration:");
    lcd.setCursor(0, 1);
    lcd.print(String(concentration) + " pcs/0.01cf");
    delay(wait_time_ms);
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  
  // End check
  lcd.clear();
  lcd.print("[Dust]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void run_light()
{
  // Begin Check
  lcd.clear();
  lcd.print("[Light]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);
  
  // Run Check
  light = analogRead(A0);
  Serial.println("Light = " + String(light));
  lcd.clear();
  lcd.print("Light:");
  lcd.setCursor(0, 1);
  lcd.print(light);
  delay(wait_time_ms);
  
  // End check
  lcd.clear();
  lcd.print("[Light]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void run_temperature()
{
  // Begin Check
  lcd.clear();
  lcd.print("[Temperature]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);
  
  // Run Check
  humidity = dht.readHumidity();
  Serial.println("Humidity = " + String(humidity));
  lcd.clear();
  lcd.print("Humidity:");
  lcd.setCursor(0, 1);
  lcd.print(humidity);
  delay(wait_time_ms);
  temp = dht.readTemperature();
  Serial.println("Temp = " + String(temp) + " Celsius");
  lcd.clear();
  lcd.print("Temp:");
  lcd.setCursor(0, 1);
  lcd.print(String(temp) + " Celsius");
  delay(wait_time_ms);
  
  // End check
  lcd.clear();
  lcd.print("[Temperature]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void run_uv()
{
  // Begin Check
  lcd.clear();
  lcd.print("[UV]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);
  
  // Run Check
  int sensorValue;
  long  sum=0;
  for(int i=0;i<1024;i++)
  {  
    sensorValue=analogRead(A1);
    sum=sensorValue+sum;
    delay(2);
  }
  sum = sum >> 10;
  // Vsig is the value of voltage measured from the SIG pin of the Grove interface
  vsig = sum * 4980.0 / 1023.0;
  Serial.println("Voltage = " + String(vsig) + "mV");
  lcd.clear();
  lcd.print("Voltage:");
  lcd.setCursor(0, 1);
  lcd.print(String(vsig) + "mV");
  delay(wait_time_ms);
  
  // End check
  lcd.clear();
  lcd.print("[UV]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void run_pulse()
{
    // Begin Check
  lcd.clear();
  lcd.print("[Pulse]");
  lcd.setCursor(0,1);
  lcd.print("begin");
  delay(wait_time_ms);
  
  // Run Check
  Serial.println("BPM = " + String(BPM));
  lcd.clear();
  lcd.print("BPM:");
  lcd.setCursor(0, 1);
  lcd.print(String(BPM));
  delay(wait_time_ms);
  
  // End check
  lcd.clear();
  lcd.print("[Pulse]");
  lcd.setCursor(0,1);
  lcd.print("end");
  delay(wait_time_ms);
}

void push_data(){
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += IP;
  cmd += "\",80";
  Serial.println(cmd);
  esp8266.println(cmd);
  delay(2000);
  if(esp8266.find("Error")){
    return;
  }
  cmd = msg ;
  cmd += "&field1=";   
  cmd += pressure;
  cmd += "&field2=";   
  cmd += altitude;
  cmd += "&field3=";   
  cmd += concentration;
  cmd += "&field4=";   
  cmd += light;
  cmd += "&field5=";   
  cmd += humidity;
  cmd += "&field6=";   
  cmd += temp;
  cmd += "&field7=";   
  cmd += vsig;
  cmd += "&field8=";   
  cmd += BPM;
  cmd += "\r\n";
  Serial.print("AT+CIPSEND=");
  esp8266.print("AT+CIPSEND=");
  Serial.println(cmd.length());
  esp8266.println(cmd.length());
  Serial.print(cmd);
  esp8266.print(cmd);
}

void interruptSetup()
{     
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 

ISR(TIMER2_COMPA_vect){                       // triggered when Timer2 counts to 124
  cli();                                      // disable interrupts while we do this
  Signal = analogRead(pulse_pin);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){      // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                         // T is the trough
      T = Signal;                            // keep track of lowest point in pulse wave 
    }
  }

  if(Signal > thresh && Signal > P){        // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                         // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when there is a pulse
      digitalWrite(blink_pin,HIGH);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realistic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time beat is found
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }   
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
      if(BPM<100){ //orange
        digitalWrite(3, HIGH);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
      }
      else if(BPM>170){ //Red
          digitalWrite(3, LOW);
          digitalWrite(4, HIGH);
          digitalWrite(5, LOW);
      }
      else{ //green
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
        
      }
    }                       
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(blink_pin,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }

  sei();     
  // enable interrupts when youre done!
}// end isr

boolean connectWiFi(){
  Serial.println("AT+CWMODE=1");
  esp8266.println("AT+CWMODE=1");
  delay(2000);
  String cmd="AT+CWJAP=\"";
  cmd+=SSID;
  cmd+="\",\"";
  cmd+=PASS;
  cmd+="\"";
  esp8266.println(cmd);
  delay(5000);
  if(esp8266.find("OK")){
    Serial.println("OK");
    return true;    
  }else{
    return false;
  }
}
