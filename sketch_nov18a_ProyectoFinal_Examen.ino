#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no activado! Activa la conexion Bluetooth.
#endif
#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include "pitches.h"
#include "esp_adc_cal.h"
#define LM35_Sensor1    35

 TaskHandle_t Task1;
TaskHandle_t Task2;

int LM35_Raw_Sensor1 = 0;
float LM35_TempC_Sensor1 = 0.0;
float LM35_TempF_Sensor1 = 0.0;
float Voltage = 0.0;
int caracterled;
char Mensajebz;
const int led_LM35 = 13;
int sensorPin = 2;
const int led_PhotoRes = 5;
int lightInit;
int lightVal;
int sensorValue;
const int led_Ble = 4;
const int BUZZZER_PIN = 18; // GIOP18 pin connected to piezo buzzer
// notes in the melody:
int melodyA[] = {
  NOTE_A4, NOTE_D4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_D4, NOTE_F4, NOTE_G4, NOTE_E4,
    NOTE_G4, NOTE_C4, NOTE_F4, NOTE_E4, NOTE_G4, NOTE_C4, NOTE_F4, NOTE_E4, NOTE_D4
};
int melodiaB[] = {
  NOTE_C4, NOTE_G3, NOTE_C4, NOTE_A3, NOTE_A3,NOTE_B3, NOTE_B3,
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurationsA[] = {
  2, 2, 4, 4, 2, 2, 4, 4, 1, 2, 2, 4, 4, 2, 2, 4, 4, 1,
};
int duracionesB[] = {
  8, 8, 3, 4, 4, 3, 3,
};
const int Pulsador1 = 14;
const int Pulsador2 = 12;
const int led_BTN1 = 26;
const int led_BTN2 = 25;
bool stadoBTN = false;
BluetoothSerial SerialBT;

void setup() {
SerialBT.begin("ESP32_Ruben");
pinMode(led_LM35, OUTPUT);
pinMode(led_PhotoRes, OUTPUT);
lightInit = analogRead(sensorPin);
pinMode(Pulsador1,INPUT_PULLUP);
pinMode(Pulsador2,INPUT_PULLUP);
pinMode(led_BTN1, OUTPUT);
pinMode(led_BTN2, OUTPUT);
pinMode(led_Ble, OUTPUT);
pinMode(BUZZZER_PIN, OUTPUT);
// We'll set up the LED pin to be an output.
  pinMode(led_PhotoRes, OUTPUT);
  lightInit = analogRead(sensorPin);
  //we will take a single reading from the light sensor and store it in the lightCal
Serial.begin(115200); 

    xTaskCreatePinnedToCore(
                    Task1code,  
                    "Task1",     
                    10000,       
                    NULL,        
                    1,           
                    &Task1,     
                    0);                            
  vTaskDelay(500); 


xTaskCreatePinnedToCore(
                    Task2code,  
                    "Task1",     
                    10000,       
                    NULL,        
                    1,           
                    &Task2,     
                    1);                            
  vTaskDelay(500); 
}


void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = analogRead(LM35_Sensor1);  
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = Voltage(mV) / 10
  LM35_TempC_Sensor1 = Voltage / 10;
  LM35_TempF_Sensor1 = (LM35_TempC_Sensor1 * 1.8) + 32;
 
  // Print The Readings
  Serial.print("Temperature = ");
  Serial.print(LM35_TempC_Sensor1);
  Serial.print(" °C , ");
  Serial.print("Temperature = ");
  Serial.print(LM35_TempF_Sensor1);
  Serial.println(" °F");
  
  delay(100);
  if(LM35_TempC_Sensor1>20){
    digitalWrite(led_LM35,HIGH);
  }if(LM35_TempC_Sensor1<19){
    digitalWrite(led_LM35,LOW);
    }
    
    ///////////////////////////FotoResistencia/////////////////////////////////////////
    sensorValue = analogRead(A0); // read analog input pin 0
  Serial.print("AnalgoRead: "); // prints the value read
  Serial.println(sensorValue, DEC); // prints the value read
  Serial.print(" \n"); // prints a space between the numbers
  delay(1000); // wait 100ms for next reading

  lightVal = analogRead(sensorPin); // read the current light levels
  Serial.print("SensorPin: "); // prints the value read
  Serial.println(lightVal); // prints the value read
  //if lightVal is less than our initial reading withing a threshold then it is dark.
  if(lightVal - lightInit <  500)
  {
      digitalWrite (led_PhotoRes, HIGH); // turn on light
  }

  //otherwise, it is bright
  else
  {
    digitalWrite (led_PhotoRes, LOW); // turn off light
  }
  /////////////////////////////////////BUZZER///////////////////////////////////////
  if (SerialBT.available()) {
    char Mensajebz = SerialBT.read();
    if (Mensajebz == 'A') {
    for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurationsA[thisNote];
    tone(BUZZZER_PIN, melodyA[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZZER_PIN);
    }
  }
  if (Mensajebz == 'B') {
    for (int nota = 0; nota < 8; nota++) {      
    int duracion = 1000 / duracionesB[nota];
    tone(BUZZZER_PIN, melodiaB[nota], duracion);
    int pausa = duracion * 1.30;
    delay(pausa);
    noTone(BUZZZER_PIN);
        }
      }
    }
  }
}

void Task2code( void * pvParameters ){
 Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
 
      for(;;){

 // Leemos el estado del GPIO    
  stadoBTN = digitalRead(Pulsador1);   
  // Controlamos el led según el resultado  
  if (stadoBTN == HIGH) {      
       digitalWrite(led_BTN1, HIGH);
       Serial.println("Boton 1 Precionado");
  }  
  else {      
       digitalWrite(led_BTN1, LOW); 
  }
// Leemos el estado del GPIO    
  stadoBTN = digitalRead(Pulsador2);   
  // Controlamos el led según el resultado  
  if (stadoBTN == HIGH) {      
       digitalWrite(led_BTN2, HIGH);
       Serial.println ("Boton 2 Precionado");
  }  
  else {      
       digitalWrite(led_BTN2, LOW); 
  }
  //Led por Bluetooth      
if(SerialBT.available()) {
  caracterled = SerialBT.read();
  
  if(caracterled == '1')
  { 
  digitalWrite(led_Ble, HIGH);
   Serial.println("Encendido");
  }
  
  if(caracterled == '0')
  { 
  digitalWrite(led_Ble, LOW);
  Serial.println("Apagado");
      } 
    } 
  }
}

void loop() {
   
}
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}
