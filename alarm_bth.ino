#include "arduinoFFT.h"
#include "FSME.h"
#include <SoftwareSerial.h>
#include <String.h>

//////////////////////////////////////////////////
enum STATES {
  OFF,
  ON,
  SIREN,

  STATES_NO
};
Transition *off_trans[1];
Transition *on_trans[2];
Transition *siren_trans[2];
State alarm_states[STATES_NO];
FSME alarm;

uint8_t isOnBtn(void);
uint8_t isOffBtn(void);
uint8_t isSirenFrec(void);
uint8_t isStopSirenBtn(void);

void onLoop(void);
void offLoop(void);
void sirenLoop(void);

//////////////////////////////////////////////////
// freq detection, used pins
uint8_t const sound_dig_pin = 2;
uint8_t const led_pin = 13;
uint8_t const sound_an_pin = A5;
uint8_t const stop_siren_btn = 5;

// freq detection, detection variables
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 128;
double const samplingFrequency = 8000;
double vReal[samples];
double vImag[samples];
uint8_t i;

// freq detection, timing variables
unsigned int const sampling_period_us = round(1000000*(1.0/samplingFrequency));;
unsigned long microseconds;

///////////////////////////////////////////////////////////////
SoftwareSerial Sim900Serial(7, 8);//Configarión de los pines serial por software
uint8_t const on_btn = 4;
uint8_t const on_led = 3;

void powerUpGprs() {
  pinMode(9, OUTPUT);
  delay(100);
  digitalWrite(9, HIGH);
  Serial.println(F("\nPower Up SIM900!"));
}

void mostrarDatosSeriales()//Muestra los datos que va entregando el sim900
{
    while(Sim900Serial.available()!=0)
        Serial.write(Sim900Serial.read());
}

void comandosAT(int salida){
    Sim900Serial.println(F("AT+CIPSTATUS"));//Consultar el estado actual de la conexión
    delay(2000);
    Sim900Serial.println(F("AT+CIPMUX=0"));//comando configura el dispositivo para una conexión IP única o múltiple 0=única
    delay(3000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CSTT=\"3g.nuevatel.com\",\"\",\"\""));//comando configura el APN, nombre de usuario y contraseña."gprs.movistar.com.ar","wap","wap"->Movistar Arg.
    delay(1000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIICR"));//REALIZAR UNA CONEXIÓN INALÁMBRICA CON GPRS O CSD
    delay(3000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIFSR"));// Obtenemos nuestra IP local
    delay(2000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIPSPRT=0"));//Establece un indicador '>' al enviar datos
    
    delay(3000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIPSTART=\"TCP\",\"18.221.56.190\",\"80\""));//Indicamos el tipo de conexión, url o dirección IP y puerto al que realizamos la conexión
    delay(6000);
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIPSEND"));//ENVÍA DATOS A TRAVÉS DE una CONEXIÓN TCP O UDP
    delay(4000);
    mostrarDatosSeriales();

    //String(latitude)
//    String datos="GET /mapacentral/api/vehiculoposition?id=12&latitud=77.1300000&longitud="+String(salida);
//http://18.221.56.190/mapacentral/api/activatealarm?id=10&status=1
    String datos="GET /mapacentral/api/activatealarm?id=10&status="+String(salida);
    Sim900Serial.println(datos);//Envía datos al servidor remoto
    delay(4000);
    mostrarDatosSeriales();
    Sim900Serial.println((char)26);
    delay(5000);//Ahora esperaremos una respuesta pero esto va a depender de las condiones de la red y este valor quizá debamos modificarlo dependiendo de las condiciones de la red
    Sim900Serial.println();
    mostrarDatosSeriales();
    Sim900Serial.println(F("AT+CIPSHUT"));//Cierra la conexión(Desactiva el contexto GPRS PDP)
    delay(5000);
    mostrarDatosSeriales();
}

void setup() {
  // put your setup code here, to run once:
  // transitions setup
  off_trans[0] = new EvnTransition(isOnBtn, ON);
  on_trans[0] = new EvnTransition(isOffBtn, OFF);
  on_trans[1] = new EvnTransition(isSirenFrec, SIREN);
  siren_trans[0] = new EvnTransition(isOffBtn, OFF);
  siren_trans[1] = new EvnTransition(isStopSirenBtn, ON);

  // states setup
  alarm_states[OFF].setState(onLoop, off_trans, 1);
  alarm_states[ON].setState(offLoop, on_trans, 2);
  alarm_states[SIREN].setState(sirenLoop, siren_trans, 2);

  // sm setup
  alarm.setStates(alarm_states, 6);
  alarm.setInitialState(OFF);
  
  ///////////////////////////////////////////////////////////////
  // freq detection, pins setup
  pinMode(sound_dig_pin, INPUT);
  pinMode(stop_siren_btn, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  
  ///////////////////////////////////////////////////////////////
  pinMode(on_btn,INPUT_PULLUP);   //boton de entrada
  pinMode(on_led, OUTPUT); //LED de Salida
  Sim900Serial.begin(9600);//Arduino se comunica con el SIM900 a una velocidad de 19200bps   
  Serial.begin(9600);//Velocidad del puerto serial de arduino
  powerUpGprs();
  delay(20000);//Tiempo prudencial para el escudo inicie sesión de red con tu operador
}

void loop() {
  // put your main code here, to run repeatedly:
  alarm.run();

  if(Sim900Serial.available())//Verificamos si hay datos disponibles desde el SIM900
    Serial.write(Sim900Serial.read());//Escribir datos
}

uint8_t isOnBtn(void) {
  if(digitalRead(on_btn) == HIGH) return 0;
  else return 1;
}
uint8_t isOffBtn(void) {
  if(digitalRead(on_btn) == HIGH) return 1;
  else return 0;
}
uint8_t isSirenFrec(void) {
  if(digitalRead(sound_dig_pin) == HIGH) {
    for(i = 0; i < samples; i++) {
      microseconds = micros();
      vReal[i] = analogRead(sound_an_pin);
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us));
    }
    
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, samples);

    double peak = FFT.MajorPeak(vReal, samples, samplingFrequency);
    if ((peak > 1800.0) && (peak < 2000.0)) return 1;
    else return 0;
  }
  else return 0;
}
uint8_t isStopSirenBtn(void) {
  if(digitalRead(stop_siren_btn) == HIGH) return 0;
  else return 1;
}

void onLoop(void) {
  if (alarm.isStateChanged()) {
    digitalWrite(on_led, HIGH);
    comandosAT(1);//Llama a la función comandosAT con salida = 1 (alerta no activada)
  }
}
void offLoop(void) {
  if (alarm.isStateChanged()) {
    digitalWrite(on_led, LOW);
    comandosAT(2);//Llama a la función comandosAT con salida = 2 (alarma apagada?)
  }
}
uint32_t last_time;
void sirenLoop(void) {
  if (alarm.isStateChanged()) {
    comandosAT(0);//Llama a la función comandosAT con salida = 0 (alerta activada)
    last_time = millis();
    digitalWrite(led_pin, HIGH);
  }
  if (millis() - last_time >= 500) {
    digitalWrite(led_pin, !digitalRead(led_pin));
    last_time = millis();
  }
}
