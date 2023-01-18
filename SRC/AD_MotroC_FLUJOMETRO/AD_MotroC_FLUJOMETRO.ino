#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//BluetoothSerial SerialBT;

// ========================================control pwm===============================================
#define MIN_DUTY_WORK  10
#define MAX_DUTY_WORK  999

float kp_B1=0.051, ki_B1=0.01, kd_B1=0.001;
const float kp_B2=0.05, ki_B2=0.01, kd_B2=0.001;
const float kp_B3=0.05, ki_B3=0.01, kd_B3=0.001;
const float kp_B4=0.05, ki_B4=0.01, kd_B4=0.001;

const float max_control_out= 300;
const float min_control_out = -300;
const float min_cumError_out = -2000;
const float max_cumError_out = 2000;
const float MIN_PWM = 0;
const float MAX_PWM = 100;

const float EMA_ALPHA_SETPOINT = 0.08;

unsigned long currentTime=0, previousTime=0;

float elapsedTime=0;
float Input_B1=0, Output_B1=0, flujo_Setpoint_B1=0, error_B1=0, lastError_B1=0, cumError_B1=0, rateError_B1=0, B1_OUT=0,Setpoint_B1_fill=0;
float Input_B2=0, Output_B2=0, flujo_Setpoint_B2=0, error_B2=0, lastError_B2=0, cumError_B2=0, rateError_B2=0, B2_OUT=0,Setpoint_B2_fill=0;
float Input_B3=0, Output_B3=0, flujo_Setpoint_B3=0, error_B3=0, lastError_B3=0, cumError_B3=0, rateError_B3=0, B3_OUT=0,Setpoint_B3_fill=0;
float Input_B4=0, Output_B4=0, flujo_Setpoint_B4=0, error_B4=0, lastError_B4=0, cumError_B4=0, rateError_B4=0, B4_OUT=0,Setpoint_B4_fill=0;


// ========================================filtro de señales===============================================
float pwm_setpont_1 = 0;
float pwm_setpont_2 = 0;
float pwm_setpont_3 = 0;
float pwm_setpont_4 = 0;

float dutycycle1_fill =0;
float dutycycle2_fill =0;
float dutycycle3_fill =0;
float dutycycle4_fill =0;

float pulsosHallHZ_1 =0;
float pulsosHallHZ_2 =0;
float pulsosHallHZ_3 =0;
float pulsosHallHZ_4 =0;


float EMA_ALPHA_DUTY_IN = 0.4;
float EMA_ALPHA_PULSO_IN = 0.1;

// ======================================VARIABLES I2C==========================================
TwoWire ESP_I2C = TwoWire(0);

#define TIERRA                    0
#define AIRE                      1
#define CALIBRACION               2
#define GUARDAR_CALIBRACION       3

#define I2C_SDA                   16 //4
#define I2C_SCL                   17 //15

byte flujo_arr[4];
byte dato_enviar[5];
float flujo_deseado_i2c=100;
int estado_drone=0;
bool pasar_pulsos = true;
bool pasar_pulsos_anterior = true;

// ======================================VARIABLES CALIBRACION==========================================
const int EEPROM_SIZE = 255;
const byte flujo_calibracion_0 = 20 ;// x10 para mmL/s
const byte flujo_calibracion_1 = 70; // x10 para mmL/s
const byte flujo_calibracion_2 = 110;// x10 para mmL/s
const byte flujo_calibracion_3 = 160; // x10 para mmL/s
const byte flujo_calibracion_satandby = 250;

const int direccion_coeficientes_PWM_FLUJO_B1 = 0;  //aumento en 16, cada estructura contiene 4 flotantes, y cad flotante 4 bytes
const int direccion_coeficientes_PWM_FLUJO_B2 = 16;
const int direccion_coeficientes_PWM_FLUJO_B3 = 32;
const int direccion_coeficientes_PWM_FLUJO_B4 = 48;

const int direccion_coeficientes_ENCODER_FLUJO_B1 = 64;
const int direccion_coeficientes_ENCODER_FLUJO_B2 = 80;
const int direccion_coeficientes_ENCODER_FLUJO_B3 = 96;
const int direccion_coeficientes_ENCODER_FLUJO_B4 = 112;

const int direccion_coeficientes_PWM_ENCODER_fake_B1 = 128;
const int direccion_coeficientes_PWM_ENCODER_fake_B2 = 144;
const int direccion_coeficientes_PWM_ENCODER_fake_B3 = 160;
const int direccion_coeficientes_PWM_ENCODER_fake_B4 = 176;

const int direccion_estado_calibracion_1 = 251;
const int direccion_estado_calibracion_2 = 252;
const int direccion_estado_calibracion_3 = 253;
const int direccion_estado_calibracion_4 = 254;

struct coeficientes{
  float A = 0;
  float B = 0;
  float C = 0;
  float D = 0;
  bool resuelta = false;  // parametro si los valores fueron encontrados
};

float flujo1,flujo2,flujo3,flujo4;

float flujo_calibracion[4] = {200,700,1100,1600};

float pulsosHallHZ_calibracion_1[4]={0,0,0,0};
float pulsosHallHZ_calibracion_2[4]={0,0,0,0};
float pulsosHallHZ_calibracion_3[4]={0,0,0,0};
float pulsosHallHZ_calibracion_4[4]={0,0,0,0};

float dutycycle_fill_calibracion_1[4]={0,0,0,0};  
float dutycycle_fill_calibracion_2[4]={0,0,0,0};  
float dutycycle_fill_calibracion_3[4]={0,0,0,0};  
float dutycycle_fill_calibracion_4[4]={0,0,0,0};  

coeficientes coeficientes_PWM_FLUJO_B1;
coeficientes coeficientes_PWM_FLUJO_B2;
coeficientes coeficientes_PWM_FLUJO_B3;
coeficientes coeficientes_PWM_FLUJO_B4;

coeficientes coeficientes_ENCODER_FLUJO_B1;
coeficientes coeficientes_ENCODER_FLUJO_B2;
coeficientes coeficientes_ENCODER_FLUJO_B3;
coeficientes coeficientes_ENCODER_FLUJO_B4;

coeficientes coeficientes_PWM_ENCODER_fake_B1;
coeficientes coeficientes_PWM_ENCODER_fake_B2;
coeficientes coeficientes_PWM_ENCODER_fake_B3;
coeficientes coeficientes_PWM_ENCODER_fake_B4;

// ======================================VARIABLES CHECK MOTORES==========================================
#define ERROR_                    0
#define OK_                       1

byte E_BOMBAS = OK_;

long ultima_vez_12c_exitoso = 0;
int count_i2c_fail = 0;

int errores_B1 = 0;
int errores_B2 = 0;
int errores_B3 = 0;
int errores_B4 = 0;
// ======================================VARIABLES TIMER==========================================
hw_timer_t * timer = NULL;//nuestra variable que representará el timer
volatile SemaphoreHandle_t timerSemaphore;//Semaforo para el control del flujo de los datos
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int cPulsosTimerD1,acumTBajoTimerD1,acumTAltoTimerD1;
volatile int cPulsosTimerD2,acumTBajoTimerD2,acumTAltoTimerD2;
volatile int cPulsosTimerD3,acumTBajoTimerD3,acumTAltoTimerD3;
volatile int cPulsosTimerD4,acumTBajoTimerD4,acumTAltoTimerD4;
volatile int cPulsosTimerHall1,cPulsosTimerHall2,cPulsosTimerHall3,cPulsosTimerHall4;
// ======================================VARIABLES TIMER==========================================


// ==================================LECTURA DEL DUTYCYCLE 1===================================
const int PIN_INPUT_DUTYCYCLE_1=39;//Pin de lectura del dutycycle del PWM
volatile int tiempoAltoD1,tiempoBajoD1,acumTAltoD1,acumTBajoD1,contPulsosD1,tiempoActualD1,tiempoAnteriorD1;
float dutycycle1,anteriorDutycycle1,cPulsosTempD1,acumTAltoTempD1,acumTBajoTempD1;
bool sinDuty1=true;
// ==================================LECTURA DEL DUTYCYCLE 1===================================

// ==================================LECTURA DEL DUTYCYCLE 2===================================
const int PIN_INPUT_DUTYCYCLE_2=36;//Pin de lectura del dutycycle del PWM
volatile int tiempoAltoD2,tiempoBajoD2,acumTAltoD2,acumTBajoD2,contPulsosD2,tiempoActualD2,tiempoAnteriorD2;
float dutycycle2,anteriorDutycycle2,cPulsosTempD2,acumTAltoTempD2,acumTBajoTempD2;
bool sinDuty2=true;
// ==================================LECTURA DEL DUTYCYCLE 2===================================

// ==================================LECTURA DEL DUTYCYCLE 3===================================
const int PIN_INPUT_DUTYCYCLE_3=35;//Pin de lectura del dutycycle del PWM
volatile int tiempoAltoD3,tiempoBajoD3,acumTAltoD3,acumTBajoD3,contPulsosD3,tiempoActualD3,tiempoAnteriorD3;
float dutycycle3,cPulsosTempD3,acumTAltoTempD3,acumTBajoTempD3;
bool sinDuty3=true;
// ==================================LECTURA DEL DUTYCYCLE 3===================================

// ==================================LECTURA DEL DUTYCYCLE 4===================================
const int PIN_INPUT_DUTYCYCLE_4=34;//Pin de lectura del dutycycle del PWM
volatile int tiempoAltoD4,tiempoBajoD4,acumTAltoD4,acumTBajoD4,contPulsosD4,tiempoActualD4,tiempoAnteriorD4;
float dutycycle4,cPulsosTempD4,acumTAltoTempD4,acumTBajoTempD4;
bool sinDuty4=true;
// ==================================LECTURA DEL DUTYCYCLE 4===================================


// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 1===================================
const int PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1=25;
const int CANAL_FREC_VAR_GEN_1=0;
const int FRECUENCIA_INICIAL_FV_1=10;
const int RESOLUCION_CANAL_FV_1=10;
float frecVarGenerada1;
// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 1===================================

// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 2===================================
const int PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2=32;
const int CANAL_FREC_VAR_GEN_2=2;
const int FRECUENCIA_INICIAL_FV_2=10;
const int RESOLUCION_CANAL_FV_2=10;
float frecVarGenerada2;
// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 2===================================

// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 2===================================
const int PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3=14;
const int CANAL_FREC_VAR_GEN_3=4;
const int FRECUENCIA_INICIAL_FV_3=10;
const int RESOLUCION_CANAL_FV_3=10;
float frecVarGenerada3;
// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 3===================================

// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 4===================================
const int PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4=27;
const int CANAL_FREC_VAR_GEN_4=6;
const int FRECUENCIA_INICIAL_FV_4=10;
const int RESOLUCION_CANAL_FV_4=10;
float frecVarGenerada4;
// ===================================GENERADOR DE ONDA CUADRADA DE FRECUENCIA VARIABLE 4===================================

// ===============================SALIDA DE PWM AL MOTOR 1=======================================
const int PIN_OUTPUT_PWM_MOTOR_1=26;
const int CANAL_PWM_MOTOR_1=8;
const int FRECUENCIA_PWM_MOTOR_1=2000;
const int RESOLUCION_CANAL_PWM_MOTOR_1=10;
float dutyMotor1=0;
// ===============================SALIDA DE PWM AL MOTOR 1=======================================

// ===============================SALIDA DE PWM AL MOTOR 2=======================================
const int PIN_OUTPUT_PWM_MOTOR_2=33;
const int CANAL_PWM_MOTOR_2=10;
const int FRECUENCIA_PWM_MOTOR_2=2000;
const int RESOLUCION_CANAL_PWM_MOTOR_2=10;
float dutyMotor2=0;
// ===============================SALIDA DE PWM AL MOTOR 2=======================================

// ===============================SALIDA DE PWM AL MOTOR 3=======================================
const int PIN_OUTPUT_PWM_MOTOR_3=22;
const int CANAL_PWM_MOTOR_3=12;
const int FRECUENCIA_PWM_MOTOR_3=2000;
const int RESOLUCION_CANAL_PWM_MOTOR_3=10;
float dutyMotor3=0;
// ===============================SALIDA DE PWM AL MOTOR 3=======================================

// ===============================SALIDA DE PWM AL MOTOR 4=======================================
const int PIN_OUTPUT_PWM_MOTOR_4=13;
const int CANAL_PWM_MOTOR_4=14;
const int FRECUENCIA_PWM_MOTOR_4=2000;
const int RESOLUCION_CANAL_PWM_MOTOR_4=10;
float dutyMotor4=0;
// ===============================SALIDA DE PWM AL MOTOR 4=======================================

//====================================LECTURA DE SENSOR HALL MOTOR1==============================
const int PIN_INPUT_HALL_1=23;
volatile int contPulsosHall1=0;
float contPulsosTempHall1,pulsosHallPorMin1;
//====================================LECTURA DE SENSOR HALL MOTOR1==============================

//====================================LECTURA DE SENSOR HALL MOTOR2==============================
const int PIN_INPUT_HALL_2=21;
volatile int contPulsosHall2=0;
float contPulsosTempHall2,pulsosHallPorMin2;
//====================================LECTURA DE SENSOR HALL MOTOR2==============================

//====================================LECTURA DE SENSOR HALL MOTOR3==============================
const int PIN_INPUT_HALL_3=19;
volatile int contPulsosHall3=0;
float contPulsosTempHall3,pulsosHallPorMin3;
//====================================LECTURA DE SENSOR HALL MOTOR3==============================

//====================================LECTURA DE SENSOR HALL MOTOR4==============================
const int PIN_INPUT_HALL_4=18;
volatile int contPulsosHall4=0;
float contPulsosTempHall4,pulsosHallPorMin4;
//====================================LECTURA DE SENSOR HALL MOTOR4==============================


// ========================================FLUJO===============================================

//================================SEGUNDO NUCLEO======================================
TaskHandle_t Task1;//Tarea para usar segundo nucleo
//================================SEGUNDO NUCLEO======================================


void ARDUINO_ISR_ATTR isrGetDuty1();//Interrupcion para detectar los tiempos alto y bajo del dutycycle enviados por el dron en la bomba 1
void ARDUINO_ISR_ATTR isrGetDuty2();//Interrupcion para detectar los tiempos alto y bajo del dutycycle enviados por el dron en la bomba 2
void ARDUINO_ISR_ATTR isrGetDuty3();//Interrupcion para detectar los tiempos alto y bajo del dutycycle enviados por el dron en la bomba 3
void ARDUINO_ISR_ATTR isrGetDuty4();//Interrupcion para detectar los tiempos alto y bajo del dutycycle enviados por el dron en la bomba 4
void ARDUINO_ISR_ATTR isrContHall1();
void ARDUINO_ISR_ATTR isrContHall2();
void ARDUINO_ISR_ATTR onTimerAlarm();//Callback del timer cuando llega a su tiempo de alarma
void setupPinInterrupcionDuty1();
void setupPinInterrupcionDuty2();
void setupPinInterrupcionDuty3();
void setupPinInterrupcionDuty4();
void setupPinesInterrupcionDuty();//Configura el puerto para detectar un evento de interrupcion de deteccion de flanco
void setupPinInterrupcionHall1();
void setupPinInterrupcionHall2();
void setupPinInterrupcionHall3();
void setupPinInterrupcionHall4();
void setupPinesInterrupcionHall();
void setupTimerAlarm();//Configura el timer de conteo de tiempo para la medicion del ciclo de trabajo
void setupGeneradorFrecuencia1();
void setupGeneradorFrecuencia2();
void setupGeneradorFrecuencia3();
void setupGeneradorFrecuencia4();
void setupGeneradoresDeFrecuencia();//Configura la salida de la onda cuadrada de frecuencia variable
void genFrecApartirDeDuty1(double duty);
void genFrecApartirDeDuty2(double duty);
void genFrecApartirDeDuty3(double duty);
void genFrecApartirDeDuty4(double duty);
void setupPwmMotor1();
void setupPwmMotor2();
void setupPwmMotor3();
void setupPwmMotor4();
void setupPwmMotores();
void generarPwmMotor1(int dutycycleMotor);
void generarPwmMotor2(int dutycycleMotor);
void generarPwmMotor3(int dutycycleMotor);
void generarPwmMotor4(int dutycycleMotor);
void calcularDutyCycle();
void setup_i2c();
void requestEvent();
void receiveEvent(int size_);
float get_flujo_total();
float leer_float_eeprom(int direccion);
bool guardar_float_eeprom(float valor_float, int direccion);
struct coeficientes obtener_coeficientes_guardar_eeprom(float x[], float y[], int direccion);
void cargar_calibracion_bombas();
String separador(String data, char separator, int index);


String separador(String data, char separator, int index){ // funcion interna para separa string, solo se sua en la funcion cargar_mapa
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void receiveEvent(int size_) {
  if (ESP_I2C.available()) { 
    byte dato_drone[2];
    ultima_vez_12c_exitoso = millis();

    for (int i=0;i<size_;i++){dato_drone[i] = ESP_I2C.read();}
    
    if (dato_drone[1] <= 255 && dato_drone[1] >= 0){
      flujo_deseado_i2c=float(dato_drone[1]);
    }
    
    if (dato_drone[0] <= 255 && dato_drone[0] >= 0){
      estado_drone=dato_drone[0];
      
      if(estado_drone == TIERRA){
        pasar_pulsos = true;
        //Serial.println("flujo deseado: " + String(flujo_deseado_i2c) +" estado dron: " + String(estado_drone));
        if(!pasar_pulsos_anterior){
          //SerialBT.println("CONFIGURACION PASAR PULSOS");
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4);
          
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_1);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_2);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_3);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_4);

          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,0);
          
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_1,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_2,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_3,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_4,0);
          
          EMA_ALPHA_PULSO_IN = 0.1;
          EMA_ALPHA_DUTY_IN = 0.4;
          E_BOMBAS = OK_;
          errores_B1 = 0;
          errores_B2 = 0;
          errores_B3 = 0;
          errores_B4 = 0;
        }
        pasar_pulsos_anterior = pasar_pulsos;
        
      }
      
      else if(estado_drone == AIRE){
        pasar_pulsos = false;
        //Serial.println("flujo deseado: " + String(flujo_deseado_i2c) +" estado dron: " + String(estado_drone));
        if(pasar_pulsos_anterior){
          //SerialBT.println("CONFIGURACION GENERAR__________________");
          ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,CANAL_FREC_VAR_GEN_1);//Se asocia el puerto 25 al pwm del canal 0
          ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,CANAL_FREC_VAR_GEN_2);//Se asocia el puerto 25 al pwm del canal 0
          ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,CANAL_FREC_VAR_GEN_3);//Se asocia el puerto 25 al pwm del canal 0
          ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,CANAL_FREC_VAR_GEN_4);//Se asocia el puerto 25 al pwm del canal 0

          ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_1,CANAL_PWM_MOTOR_1);
          ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_2,CANAL_PWM_MOTOR_2);
          ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_3,CANAL_PWM_MOTOR_3);
          ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_4,CANAL_PWM_MOTOR_4);
          EMA_ALPHA_PULSO_IN = 0.1;
          EMA_ALPHA_DUTY_IN = 0.4;
          errores_B1 = 0;
          errores_B2 = 0;
          errores_B3 = 0;
          errores_B4 = 0;
        }
        pasar_pulsos_anterior = pasar_pulsos;
        
      }
      
      else if(estado_drone == CALIBRACION){
        if (pasar_pulsos){ // if para no calibrar si el dron esta en aire
          EMA_ALPHA_PULSO_IN = 0.01;
          EMA_ALPHA_DUTY_IN = 0.01;
          //SerialBT.println("CALIBRANDO");
          if(flujo_deseado_i2c == flujo_calibracion_0){
            //SerialBT.println("GUARDADO FLUJO CALIBRACION A");
            pulsosHallHZ_calibracion_1[0] = pulsosHallHZ_1;
            pulsosHallHZ_calibracion_2[0] = pulsosHallHZ_2;
            pulsosHallHZ_calibracion_3[0] = pulsosHallHZ_3;
            pulsosHallHZ_calibracion_4[0] = pulsosHallHZ_4;
            dutycycle_fill_calibracion_1[0] = dutycycle1_fill;
            dutycycle_fill_calibracion_2[0] = dutycycle2_fill;
            dutycycle_fill_calibracion_3[0] = dutycycle3_fill;
            dutycycle_fill_calibracion_4[0] = dutycycle4_fill;
            //SerialBT.print(String(pulsosHallHZ_calibracion_1[0])+ "," +String(pulsosHallHZ_calibracion_2[0])+ "," +String(pulsosHallHZ_calibracion_3[0])+ "," +String(pulsosHallHZ_calibracion_4[0])+ "    " +String(dutycycle_fill_calibracion_1[0])+ "," +String(dutycycle_fill_calibracion_2[0])+ "," +String(dutycycle_fill_calibracion_3[0])+ "," +String(dutycycle_fill_calibracion_4[0]));
            ////SerialBT.println("  " +String(-0.0084*dutycycle1_fill*dutycycle1_fill*dutycycle1_fill  +  1.24*dutycycle1_fill*dutycycle1_fill  -  0.7025*dutycycle1_fill) + "," +String(-0.0084*dutycycle2_fill*dutycycle2_fill*dutycycle2_fill  +  1.24*dutycycle2_fill*dutycycle2_fill  -  0.7025*dutycycle2_fill)+ "," +String(-0.0084*dutycycle3_fill*dutycycle3_fill*dutycycle3_fill  +  1.24*dutycycle3_fill*dutycycle3_fill  -  0.7025*dutycycle3_fill) + "," +String(-0.0084*dutycycle4_fill*dutycycle4_fill*dutycycle4_fill  +  1.24*dutycycle4_fill*dutycycle4_fill  -  0.7025*dutycycle4_fill));
          } else if(flujo_deseado_i2c == flujo_calibracion_1){
            //SerialBT.println("GUARDADO FLUJO CALIBRACION B");
            pulsosHallHZ_calibracion_1[1] = pulsosHallHZ_1;
            pulsosHallHZ_calibracion_2[1] = pulsosHallHZ_2;
            pulsosHallHZ_calibracion_3[1] = pulsosHallHZ_3;
            pulsosHallHZ_calibracion_4[1] = pulsosHallHZ_4;
            dutycycle_fill_calibracion_1[1] = dutycycle1_fill;
            dutycycle_fill_calibracion_2[1] = dutycycle2_fill;
            dutycycle_fill_calibracion_3[1] = dutycycle3_fill;
            dutycycle_fill_calibracion_4[1] = dutycycle4_fill;
            //SerialBT.print(String(pulsosHallHZ_calibracion_1[1])+ "," +String(pulsosHallHZ_calibracion_2[1])+ "," +String(pulsosHallHZ_calibracion_3[1])+ "," +String(pulsosHallHZ_calibracion_4[1])+ "    " +String(dutycycle_fill_calibracion_1[1])+ "," +String(dutycycle_fill_calibracion_2[1])+ "," +String(dutycycle_fill_calibracion_3[1])+ "," +String(dutycycle_fill_calibracion_4[1]));
            ////SerialBT.println("  " +String(-0.0084*dutycycle1_fill*dutycycle1_fill*dutycycle1_fill  +  1.24*dutycycle1_fill*dutycycle1_fill  -  0.7025*dutycycle1_fill) + "," +String(-0.0084*dutycycle2_fill*dutycycle2_fill*dutycycle2_fill  +  1.24*dutycycle2_fill*dutycycle2_fill  -  0.7025*dutycycle2_fill)+ "," +String(-0.0084*dutycycle3_fill*dutycycle3_fill*dutycycle3_fill  +  1.24*dutycycle3_fill*dutycycle3_fill  -  0.7025*dutycycle3_fill) + "," +String(-0.0084*dutycycle4_fill*dutycycle4_fill*dutycycle4_fill  +  1.24*dutycycle4_fill*dutycycle4_fill  -  0.7025*dutycycle4_fill));
          }else if(flujo_deseado_i2c == flujo_calibracion_2){
            //SerialBT.println("GUARDADO FLUJO CALIBRACION C");
            pulsosHallHZ_calibracion_1[2] = pulsosHallHZ_1;
            pulsosHallHZ_calibracion_2[2] = pulsosHallHZ_2;
            pulsosHallHZ_calibracion_3[2] = pulsosHallHZ_3;
            pulsosHallHZ_calibracion_4[2] = pulsosHallHZ_4;
            dutycycle_fill_calibracion_1[2] = dutycycle1_fill;
            dutycycle_fill_calibracion_2[2] = dutycycle2_fill;
            dutycycle_fill_calibracion_3[2] = dutycycle3_fill;
            dutycycle_fill_calibracion_4[2] = dutycycle4_fill;
            //SerialBT.print(String(pulsosHallHZ_calibracion_1[2])+ "," +String(pulsosHallHZ_calibracion_2[2])+ "," +String(pulsosHallHZ_calibracion_3[2])+ "," +String(pulsosHallHZ_calibracion_4[2])+ "    " +String(dutycycle_fill_calibracion_1[2])+ "," +String(dutycycle_fill_calibracion_2[2])+ "," +String(dutycycle_fill_calibracion_3[2])+ "," +String(dutycycle_fill_calibracion_4[2]));
            ////SerialBT.println("  " +String(-0.0084*dutycycle1_fill*dutycycle1_fill*dutycycle1_fill  +  1.24*dutycycle1_fill*dutycycle1_fill  -  0.7025*dutycycle1_fill) + "," +String(-0.0084*dutycycle2_fill*dutycycle2_fill*dutycycle2_fill  +  1.24*dutycycle2_fill*dutycycle2_fill  -  0.7025*dutycycle2_fill)+ "," +String(-0.0084*dutycycle3_fill*dutycycle3_fill*dutycycle3_fill  +  1.24*dutycycle3_fill*dutycycle3_fill  -  0.7025*dutycycle3_fill) + "," +String(-0.0084*dutycycle4_fill*dutycycle4_fill*dutycycle4_fill  +  1.24*dutycycle4_fill*dutycycle4_fill  -  0.7025*dutycycle4_fill));

          }else if(flujo_deseado_i2c == flujo_calibracion_3){
            //SerialBT.println("GUARDADO FLUJO CALIBRACION D");
            pulsosHallHZ_calibracion_1[3] = pulsosHallHZ_1;
            pulsosHallHZ_calibracion_2[3] = pulsosHallHZ_2;
            pulsosHallHZ_calibracion_3[3] = pulsosHallHZ_3;
            pulsosHallHZ_calibracion_4[3] = pulsosHallHZ_4;
            dutycycle_fill_calibracion_1[3] = dutycycle1_fill;
            dutycycle_fill_calibracion_2[3] = dutycycle2_fill;
            dutycycle_fill_calibracion_3[3] = dutycycle3_fill;
            dutycycle_fill_calibracion_4[3] = dutycycle4_fill;
            //SerialBT.print(String(pulsosHallHZ_calibracion_1[3])+ "," +String(pulsosHallHZ_calibracion_2[3])+ "," +String(pulsosHallHZ_calibracion_3[3])+ "," +String(pulsosHallHZ_calibracion_4[3])+ "    " +String(dutycycle_fill_calibracion_1[3])+ "," +String(dutycycle_fill_calibracion_2[3])+ "," +String(dutycycle_fill_calibracion_3[3])+ "," +String(dutycycle_fill_calibracion_4[3]));
            ////SerialBT.println("  " +String(-0.0084*dutycycle1_fill*dutycycle1_fill*dutycycle1_fill  +  1.24*dutycycle1_fill*dutycycle1_fill  -  0.7025*dutycycle1_fill) + "," +String(-0.0084*dutycycle2_fill*dutycycle2_fill*dutycycle2_fill  +  1.24*dutycycle2_fill*dutycycle2_fill  -  0.7025*dutycycle2_fill)+ "," +String(-0.0084*dutycycle3_fill*dutycycle3_fill*dutycycle3_fill  +  1.24*dutycycle3_fill*dutycycle3_fill  -  0.7025*dutycycle3_fill) + "," +String(-0.0084*dutycycle4_fill*dutycycle4_fill*dutycycle4_fill  +  1.24*dutycycle4_fill*dutycycle4_fill  -  0.7025*dutycycle4_fill));

          }else if(flujo_deseado_i2c == flujo_calibracion_satandby){
            //SerialBT.println("ESPERANDO FLUJO DE CALIBRACION");
          }else{
            //SerialBT.println("DATO NO VALIDO PARA CALIBRACION");
          }
        }
        
      }
      
      else if(estado_drone == GUARDAR_CALIBRACION){
        if (pasar_pulsos){ // if para no calibrar si el dron esta en aire
          // CALCULAR ECUACIONES PWM --- FLUJO
          
          errores_B1 = 0;
          errores_B2 = 0;
          errores_B3 = 0;
          errores_B4 = 0;
          
          coeficientes_PWM_FLUJO_B1 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_1, flujo_calibracion, direccion_coeficientes_PWM_FLUJO_B1);
          coeficientes_PWM_FLUJO_B2 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_2, flujo_calibracion, direccion_coeficientes_PWM_FLUJO_B2);
          coeficientes_PWM_FLUJO_B3 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_3, flujo_calibracion, direccion_coeficientes_PWM_FLUJO_B3);
          coeficientes_PWM_FLUJO_B4 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_4, flujo_calibracion, direccion_coeficientes_PWM_FLUJO_B4);

          // CALCULAR ECUACIONES ENCODER --- FLUJO
          coeficientes_ENCODER_FLUJO_B1 = obtener_coeficientes_guardar_eeprom (pulsosHallHZ_calibracion_1, flujo_calibracion, direccion_coeficientes_ENCODER_FLUJO_B1);
          coeficientes_ENCODER_FLUJO_B2 = obtener_coeficientes_guardar_eeprom (pulsosHallHZ_calibracion_2, flujo_calibracion, direccion_coeficientes_ENCODER_FLUJO_B2);
          coeficientes_ENCODER_FLUJO_B3 = obtener_coeficientes_guardar_eeprom (pulsosHallHZ_calibracion_3, flujo_calibracion, direccion_coeficientes_ENCODER_FLUJO_B3);
          coeficientes_ENCODER_FLUJO_B4 = obtener_coeficientes_guardar_eeprom (pulsosHallHZ_calibracion_4, flujo_calibracion, direccion_coeficientes_ENCODER_FLUJO_B4);

          // CALCULAR ECUACIONES PWM --- ENCODER FAKE
          
          coeficientes_PWM_ENCODER_fake_B1 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_1, pulsosHallHZ_calibracion_1, direccion_coeficientes_PWM_ENCODER_fake_B1);
          coeficientes_PWM_ENCODER_fake_B2 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_2, pulsosHallHZ_calibracion_2, direccion_coeficientes_PWM_ENCODER_fake_B2);
          coeficientes_PWM_ENCODER_fake_B3 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_3, pulsosHallHZ_calibracion_3, direccion_coeficientes_PWM_ENCODER_fake_B3);
          coeficientes_PWM_ENCODER_fake_B4 = obtener_coeficientes_guardar_eeprom (dutycycle_fill_calibracion_4, pulsosHallHZ_calibracion_4, direccion_coeficientes_PWM_ENCODER_fake_B4);

          if(coeficientes_PWM_FLUJO_B1.resuelta && coeficientes_ENCODER_FLUJO_B1.resuelta && coeficientes_PWM_ENCODER_fake_B1.resuelta){
            //SerialBT.println("calibracion M1 guardada");
            EEPROM.write(direccion_estado_calibracion_1, 1); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
          }else{
            EEPROM.write(direccion_estado_calibracion_1, 0); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
            //SerialBT.println("calibracion M1 NO guardada");
          }

          if(coeficientes_PWM_FLUJO_B2.resuelta && coeficientes_ENCODER_FLUJO_B2.resuelta && coeficientes_PWM_ENCODER_fake_B2.resuelta){
            //SerialBT.println("calibracion M2 guardada");
            EEPROM.write(direccion_estado_calibracion_2, 1); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
          }else{
            EEPROM.write(direccion_estado_calibracion_2, 0); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
            //SerialBT.println("calibracion M2 NO guardada");
          }

          if(coeficientes_PWM_FLUJO_B3.resuelta && coeficientes_ENCODER_FLUJO_B3.resuelta && coeficientes_PWM_ENCODER_fake_B3.resuelta){
            //SerialBT.println("calibracion M3 guardada");
            EEPROM.write(direccion_estado_calibracion_3, 1); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
          }else{
            EEPROM.write(direccion_estado_calibracion_3, 0); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
            //SerialBT.println("calibracion M3 NO guardada");
          }

          if(coeficientes_PWM_FLUJO_B4.resuelta && coeficientes_ENCODER_FLUJO_B4.resuelta && coeficientes_PWM_ENCODER_fake_B4.resuelta){
            //SerialBT.println("calibracion M4 guardada");
            EEPROM.write(direccion_estado_calibracion_4, 1); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
          }else{
            EEPROM.write(direccion_estado_calibracion_4, 0); // se guarda el estado de 1 para saber si ya se subio una calibracion y asi cargarla al iniciar
            EEPROM.commit();
            //SerialBT.println("calibracion M4 NO guardada");
          }

          cargar_calibracion_bombas();
          
        }
      }
    }
  }
}

void requestEvent() {
  ultima_vez_12c_exitoso = millis();
  float flujo_enviar;
  if(estado_drone == CALIBRACION){
    if(flujo_deseado_i2c == flujo_calibracion_0){
      flujo_enviar = pulsosHallHZ_1;
      byte * flujo_arr = (byte *) &flujo_enviar;
      ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
      
    }else if(flujo_deseado_i2c == flujo_calibracion_1){
      flujo_enviar = pulsosHallHZ_2;
      byte * flujo_arr = (byte *) &flujo_enviar;
      ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
      
    }else if(flujo_deseado_i2c == flujo_calibracion_2){
      flujo_enviar = pulsosHallHZ_3;
      byte * flujo_arr = (byte *) &flujo_enviar;
      ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
      
    }else if(flujo_deseado_i2c == flujo_calibracion_3){
      flujo_enviar = pulsosHallHZ_4;
      byte * flujo_arr = (byte *) &flujo_enviar;
      ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
      
    }else{
      flujo_enviar = -1;                                              // valor para informar a la app que se esta a espera de la confirmacion para calibrar
      byte * flujo_arr = (byte *) &flujo_enviar;
      ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
    }
  }
  else if (estado_drone == GUARDAR_CALIBRACION){
    flujo_enviar = -100;
    byte * flujo_arr = (byte *) &flujo_enviar;
    ESP_I2C.write(flujo_arr,sizeof(flujo_arr));
  }
  else{
    flujo_enviar = get_flujo_total();
    byte * flujo_arr = (byte *) &flujo_enviar;
    dato_enviar[0] = flujo_arr[0];
    dato_enviar[1] = flujo_arr[1];
    dato_enviar[2] = flujo_arr[2];
    dato_enviar[3] = flujo_arr[3];
    dato_enviar[4] = E_BOMBAS;
    ESP_I2C.write(dato_enviar,sizeof(dato_enviar));
    //Serial.println("flujo BOMBAS: " + String(flujo_enviar) +" estado BOMBAS: " + String(E_BOMBAS));
  }
      //SerialBT.print("e 1:" + String(error_B1) + " 2:" + String(error_B2) + " 3:" + String(error_B3) + " 4:" + String(error_B4));
      //SerialBT.print(" h 1:" + String(Input_B1) + " 2:" + String(Input_B2) + " 3:" + String(Input_B3) + " 4:" + String(Input_B4));
      //SerialBT.println(" f 1:" + String(flujo_Setpoint_B1) + " 2:" + String(flujo_Setpoint_B2) + " 3:" + String(flujo_Setpoint_B3) + " 4:" + String(flujo_Setpoint_B4) + " ZONA:" + String(flujo_deseado_i2c));

}

void setup_i2c(){
  ESP_I2C.begin(uint8_t(8),I2C_SDA, I2C_SCL, uint32_t (100000));
  ESP_I2C.onRequest(requestEvent); 
  ESP_I2C.onReceive(receiveEvent); 
  ESP_I2C.setTimeOut(1);
}

void computePID(){
  if ((millis()-previousTime) > 10){
    if(!pasar_pulsos){
      Setpoint_B1_fill = EMA_ALPHA_SETPOINT * (dutycycle1_fill) + (1 - EMA_ALPHA_SETPOINT) * Setpoint_B1_fill;
      Setpoint_B2_fill = EMA_ALPHA_SETPOINT * (dutycycle2_fill) + (1 - EMA_ALPHA_SETPOINT) * Setpoint_B2_fill;
      Setpoint_B3_fill = EMA_ALPHA_SETPOINT * (dutycycle3_fill) + (1 - EMA_ALPHA_SETPOINT) * Setpoint_B3_fill;
      Setpoint_B4_fill = EMA_ALPHA_SETPOINT * (dutycycle4_fill) + (1 - EMA_ALPHA_SETPOINT) * Setpoint_B4_fill;
      
      flujo_Setpoint_B1 = coeficientes_PWM_FLUJO_B1.C * Setpoint_B1_fill + coeficientes_PWM_FLUJO_B1.D;  //coeficientes_PWM_FLUJO_B1.A * (Setpoint_B1_fill*Setpoint_B1_fill*Setpoint_B1_fill) + coeficientes_PWM_FLUJO_B1.B * (Setpoint_B1_fill*Setpoint_B1_fill) + coeficientes_PWM_FLUJO_B1.C * Setpoint_B1_fill + coeficientes_PWM_FLUJO_B1.D;
      flujo_Setpoint_B2 = coeficientes_PWM_FLUJO_B2.C * Setpoint_B2_fill + coeficientes_PWM_FLUJO_B2.D;  //coeficientes_PWM_FLUJO_B2.A * (Setpoint_B2_fill*Setpoint_B2_fill*Setpoint_B2_fill) + coeficientes_PWM_FLUJO_B2.B * (Setpoint_B2_fill*Setpoint_B2_fill) + coeficientes_PWM_FLUJO_B2.C * Setpoint_B2_fill + coeficientes_PWM_FLUJO_B2.D;
      flujo_Setpoint_B3 = coeficientes_PWM_FLUJO_B3.C * Setpoint_B3_fill + coeficientes_PWM_FLUJO_B3.D;  //coeficientes_PWM_FLUJO_B3.A * (Setpoint_B3_fill*Setpoint_B3_fill*Setpoint_B3_fill) + coeficientes_PWM_FLUJO_B3.B * (Setpoint_B3_fill*Setpoint_B3_fill) + coeficientes_PWM_FLUJO_B3.C * Setpoint_B3_fill + coeficientes_PWM_FLUJO_B3.D;
      flujo_Setpoint_B4 = coeficientes_PWM_FLUJO_B4.C * Setpoint_B4_fill + coeficientes_PWM_FLUJO_B4.D;  //coeficientes_PWM_FLUJO_B4.A * (Setpoint_B4_fill*Setpoint_B4_fill*Setpoint_B4_fill) + coeficientes_PWM_FLUJO_B4.B * (Setpoint_B4_fill*Setpoint_B4_fill) + coeficientes_PWM_FLUJO_B4.C * Setpoint_B4_fill + coeficientes_PWM_FLUJO_B4.D;            

      flujo_Setpoint_B1 = constrain(flujo_Setpoint_B1,0,1800);
      flujo_Setpoint_B2 = constrain(flujo_Setpoint_B2,0,1800);
      flujo_Setpoint_B3 = constrain(flujo_Setpoint_B3,0,1800);
      flujo_Setpoint_B4 = constrain(flujo_Setpoint_B4,0,1800);
      
      flujo_Setpoint_B1 = flujo_Setpoint_B1*(flujo_deseado_i2c/100);
      flujo_Setpoint_B2 = flujo_Setpoint_B2*(flujo_deseado_i2c/100);
      flujo_Setpoint_B3 = flujo_Setpoint_B3*(flujo_deseado_i2c/100);
      flujo_Setpoint_B4 = flujo_Setpoint_B4*(flujo_deseado_i2c/100);

      
      error_B1 = flujo_Setpoint_B1 - Input_B1;
      cumError_B1 += error_B1; 
      rateError_B1 = (error_B1 - lastError_B1);
    
      error_B2 = flujo_Setpoint_B2 - Input_B2;
      cumError_B2 += error_B2;
      rateError_B2 = (error_B2 - lastError_B2);
    
      error_B3 = flujo_Setpoint_B3 - Input_B3;
      cumError_B3 += error_B3; 
      rateError_B3 = (error_B3 - lastError_B3);
    
      error_B4 = flujo_Setpoint_B4 - Input_B4;
      cumError_B4 += error_B4;
      rateError_B4 = (error_B4 - lastError_B4);
      
  
      if (cumError_B1 >= max_cumError_out){
        cumError_B1 = max_cumError_out;
        errores_B1++;        
      }else{errores_B1 = 0;}
      
      if (cumError_B2 >= max_cumError_out){
        cumError_B2 = max_cumError_out;
        errores_B2++;
      }else{errores_B2 = 0;}
      
      if (cumError_B3 >= max_cumError_out){
        cumError_B3 = max_cumError_out;
        errores_B3++;
        //SerialBT.println("error b3 " + String(errores_B3));
      }else{errores_B3 = 0;}
      
      if (cumError_B4 >= max_cumError_out){
        cumError_B4 = max_cumError_out;
        errores_B4++;
      }else{errores_B4 = 0;}

  
      if (cumError_B1 <= min_cumError_out){
        cumError_B1 = min_cumError_out;
        errores_B1++;
      }else{errores_B1 = 0;}
      
      if (cumError_B2 <= min_cumError_out){
        cumError_B2 = min_cumError_out;
        errores_B2++;
      }else{errores_B2 = 0;}
      
      if (cumError_B3 <= min_cumError_out){
        cumError_B3 = min_cumError_out;
        errores_B3++;
        //SerialBT.println("error b3 " + String(errores_B3));
      }else{errores_B3 = 0;}
      
      if (cumError_B4 <= min_cumError_out){
        cumError_B4 = min_cumError_out;
        errores_B4++;
      }else{errores_B4 = 0;}

      if(errores_B1 > 100 || errores_B2 > 100 || errores_B3 > 100 || errores_B4 > 100){E_BOMBAS = ERROR_;}
      else{E_BOMBAS = OK_;}
      
      Output_B1 = kp_B1*error_B1 + ki_B1*cumError_B1 + kd_B1*rateError_B1; 
      Output_B2 = kp_B1*error_B2 + ki_B1*cumError_B2 + kd_B1*rateError_B2;
      Output_B3 = kp_B1*error_B3 + ki_B1*cumError_B3 + kd_B1*rateError_B3;
      Output_B4 = kp_B1*error_B4 + ki_B1*cumError_B4 + kd_B1*rateError_B4;
  
      if (Output_B1 > max_control_out){Output_B1 = max_control_out;}
      if (Output_B2 > max_control_out){Output_B2 = max_control_out;}
      if (Output_B3 > max_control_out){Output_B3 = max_control_out;}
      if (Output_B4 > max_control_out){Output_B4 = max_control_out;}
  
      if (Output_B1 < min_control_out){Output_B1 = min_control_out;}
      if (Output_B2 < min_control_out){Output_B2 = min_control_out;}
      if (Output_B3 < min_control_out){Output_B3 = min_control_out;}
      if (Output_B4 < min_control_out){Output_B4 = min_control_out;}
     
      lastError_B1 = error_B1; 
      lastError_B2 = error_B2;
      lastError_B3 = error_B3;
      lastError_B4 = error_B4;
      
      float PWM_base_B1 = Setpoint_B1_fill*(flujo_deseado_i2c/100);
      float PWM_base_B2 = Setpoint_B2_fill*(flujo_deseado_i2c/100);
      float PWM_base_B3 = Setpoint_B3_fill*(flujo_deseado_i2c/100);
      float PWM_base_B4 = Setpoint_B4_fill*(flujo_deseado_i2c/100);
  
      B1_OUT = Output_B1 + PWM_base_B1;
      B2_OUT = Output_B2 + PWM_base_B2;
      B3_OUT = Output_B3 + PWM_base_B3;
      B4_OUT = Output_B4 + PWM_base_B4;
  
      if (B1_OUT > MAX_PWM){B1_OUT = MAX_PWM;}
      if (B2_OUT > MAX_PWM){B2_OUT = MAX_PWM;}
      if (B3_OUT > MAX_PWM){B3_OUT = MAX_PWM;}
      if (B4_OUT > MAX_PWM){B4_OUT = MAX_PWM;}
      
      if (B1_OUT < MIN_PWM){B1_OUT = MIN_PWM;}
      if (B2_OUT < MIN_PWM){B2_OUT = MIN_PWM;}
      if (B3_OUT < MIN_PWM){B3_OUT = MIN_PWM;}
      if (B4_OUT < MIN_PWM){B4_OUT = MIN_PWM;}
      
      //SerialBT.println(String(flujo_Setpoint_B1) + "," + String(Input_B1) + "," + String(error_B1)+ "," + String(Output_B1)+ "," + String(PWM_base_B1)+ "," + String(B1_OUT));
      //Serial.print("error : ");
      //Serial.print(error_B1);     //  flujo real
      //Serial.print(",");
      //Serial.print("  flujo real: ");
      //Serial.print(Input_B1);     //  flujo real
      //Serial.print(",");
      //Serial.print("  PWM deseado in: ");
      //Serial.print(Setpoint_B1_fill); // flujo flujo calculado deseado con base en pwm y mapa
      //Serial.print(" error acumulado: ");
      //Serial.print(cumError_B1); // flujo flujo deseado calculado con la funcion
      //Serial.print(" salida de control: ");
      //Serial.print(B1_OUT); // salida de control
      //Serial.println();

      //if(E_BOMBAS == OK_){
        generarPwmMotor1(B1_OUT*10);
        generarPwmMotor2(B2_OUT*10);
        generarPwmMotor3(B3_OUT*10);
        generarPwmMotor4(B4_OUT*10);
      //}else{
      //  generarPwmMotor1(0);
      //  generarPwmMotor2(0);
      //  generarPwmMotor3(0);
      //  generarPwmMotor4(0);
      //}
    }
    
    previousTime =  millis();
  }
}

struct coeficientes obtener_coeficientes_guardar_eeprom(float x_[], float y_[], int direccion){
    
  double x[2]={x_[0],x_[3]}; 
  double y[2]={y_[0],y_[3]}; 
  struct coeficientes co;
  
  if((x[0]+x[1]+x[2]+x[3]) > 0.001 && (y[0]+y[1]+y[2]+y[3]) > 0.001 ){ // se asegura que los datos no sean 0 para que no falle el calculo
    int i,j,k,n=1,N=2;   
    double X[2*n+1];                                                          //Array que guardara el valor de sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    
    //SerialBT.println("MODELANDO SISTEMA, GRADO: " +String(n)+"    Numero de datos: "+String(N));        //tamaño del arrray de x, y     cin>>N;

    for (i=0;i<2*n+1;i++){
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //se guaradaran en el array, consecutivamente
    }
    
    double B[n+1][n+2],a[n+1];            //B es la Normal matrix(augmented), que guardara la ecuacion, 'a' es el valor de la ceificiente final
    
    for (i=0;i<=n;i++){
        for (j=0;j<=n;j++){
            B[i][j]=X[i+j];}            //Construlle la matriz Normal Almacenando los coeficientes correspondientes en las posiciones correctas excepto la última columna de la matriz    
    }
    
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    
    for (i=0;i<n+1;i++){
        Y[i]=0;
        for (j=0;j<N;j++){
        Y[i]=Y[i]+pow(x[j],i)*y[j];}        //posiciones se guardaran en sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    
    for (i=0;i<=n;i++){
        B[i][n+1]=Y[i];}                //Cargar los valores de Y como la última columna de B (Matriz Normal pero aumentada)
    
    n=n+1;                //N se hace n + 1 porque la parte de Eliminación Gaussiana de abajo es para n ecuaciones, pero aquí n es el grado de polinomio y para n grado obtendremos n + 1 ecuaciones
    
    //Serial.println("La Normal(Augmented Matrix) es: ");
    
    //for (i=0;i<n;i++){            //imprime la Normal-augmented matrix
    //    for (j=0;j<=n;j++)
    //        //Serial.println(B[i][j]);
    //}
    
    for (i=0;i<n;i++){                    //Inicia eliminacion de gauss
        for (k=i+1;k<n;k++){
            if (B[i][i]<B[k][i]){
                for (j=0;j<=n;j++){
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
            }
        }
    }
      
    for (i=0;i<n-1;i++){            //loop para hacer la eliminacio de gauss
        for (k=i+1;k<n;k++){
          double t=B[k][i]/B[i][i];
          for (j=0;j<=n;j++){                  
            B[k][j]=B[k][j]-t*B[i][j];    //Hacer que los elementos por debajo de los elementos de pivote sean iguales a cero o eliminar las variables             
          }
        }
    }
    
    for (i=n-1;i>=0;i--){               //Sustitución posterior   //X es una matriz cuyos valores corresponden a los valores de x, y, z ..
      a[i]=B[i][n];                     //Hacer que la variable sea calculada igual a la de la última ecuación
      for (j=0;j<n;j++){
        if (j!=i){                      //Luego resta todos los valores de lhs excepto el coeficiente de la variable cuyo valor                                  is being calculated
          a[i]=a[i]-B[i][j]*a[j];}
      }
      a[i]=a[i]/B[i][i];                //Ahora finalmente dividir el rhs por el coeficiente de la variable a calcular
    }

    bool a_resuelta,b_resuelta,c_resuelta,d_resuelta;
    
    if(isnan(a[0])){
      co.D = 0;
      d_resuelta = false;
    }else{
      co.D = a[0];
      d_resuelta = true;
    }

    if(isnan(a[1])){
      co.C = 0;
      c_resuelta = false;
    }else{
      co.C = a[1];
      c_resuelta = true;
    }
  
    b_resuelta = true;
    co.B = 0;// a[2];
    a_resuelta = true;
    co.A = 0;// a[3];

    if(a_resuelta && b_resuelta && c_resuelta && d_resuelta){
      co.resuelta = true;
      guardar_float_eeprom  (float(co.A), direccion);
      guardar_float_eeprom  (float(co.B), direccion + 4);
      guardar_float_eeprom  (float(co.C), direccion + 8);
      guardar_float_eeprom  (float(co.D), direccion + 12);
      //SerialBT.println("El valor de la coeficiente es:");
      //SerialBT.println( "y = " + String(co.B) + " X^2 + " + String(co.C) + " X + " + String(co.D)); 
    }else{co.resuelta = false;}
    return co;
  }else{
    co.A=0;
    co.B=0;
    co.C=0;
    co.D=0;
    co.resuelta = false;
    return co;
  }
}

void cargar_calibracion_bombas(){
  Serial.println("cargando calibracion de bombas");
  byte estado_calibracion_1 = EEPROM.read(direccion_estado_calibracion_1);
  byte estado_calibracion_2 = EEPROM.read(direccion_estado_calibracion_2);
  byte estado_calibracion_3 = EEPROM.read(direccion_estado_calibracion_3);
  byte estado_calibracion_4 = EEPROM.read(direccion_estado_calibracion_4);
  
  if(estado_calibracion_1 ==  1){
    coeficientes_PWM_FLUJO_B1.A = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B1);
    coeficientes_PWM_FLUJO_B1.B = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B1 + 4);
    coeficientes_PWM_FLUJO_B1.C = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B1 + 8);
    coeficientes_PWM_FLUJO_B1.D = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B1 + 12);

    coeficientes_ENCODER_FLUJO_B1.A = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B1);
    coeficientes_ENCODER_FLUJO_B1.B = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B1 + 4);
    coeficientes_ENCODER_FLUJO_B1.C = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B1 + 8);
    coeficientes_ENCODER_FLUJO_B1.D = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B1 + 12);

    coeficientes_PWM_ENCODER_fake_B1.A = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B1);
    coeficientes_PWM_ENCODER_fake_B1.B = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B1 + 4);
    coeficientes_PWM_ENCODER_fake_B1.C = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B1 + 8);
    coeficientes_PWM_ENCODER_fake_B1.D = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B1 + 12);
    
    //SerialBT.println("CALIBRACION M1 CARGADA DESDE EEPROM");
    //SerialBT.println("PWM-FLUJO-B1-A: " + String(coeficientes_PWM_FLUJO_B1.A)+" PWM-FLUJO-B1-B: " + String(coeficientes_PWM_FLUJO_B1.B)+" PWM-FLUJO-B1-C: " + String(coeficientes_PWM_FLUJO_B1.C)+" PWM-FLUJO-B1-A: " + String(coeficientes_PWM_FLUJO_B1.D));
    //SerialBT.println("HALL-FLUJO-B1-A: " + String(coeficientes_ENCODER_FLUJO_B1.A)+" HALL-FLUJO-B1-B: " + String(coeficientes_ENCODER_FLUJO_B1.B)+" HALL-FLUJO-B1-C: " + String(coeficientes_ENCODER_FLUJO_B1.C)+" HALL-FLUJO-B1-D: " + String(coeficientes_ENCODER_FLUJO_B1.D));
    //SerialBT.println("PWM-HALL-B1-A: " + String(coeficientes_PWM_ENCODER_fake_B1.A)+" PWM-HALL-B1-B: " + String(coeficientes_PWM_ENCODER_fake_B1.B)+" PWM-HALL-B1-C: " + String(coeficientes_PWM_ENCODER_fake_B1.C)+" PWM-HALL-B1-D: " + String(coeficientes_PWM_ENCODER_fake_B1.D));

  }else{E_BOMBAS = ERROR_;}
  
  if(estado_calibracion_2 ==  1){
    coeficientes_PWM_FLUJO_B2.A = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B2);
    coeficientes_PWM_FLUJO_B2.B = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B2 + 4);
    coeficientes_PWM_FLUJO_B2.C = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B2 + 8);
    coeficientes_PWM_FLUJO_B2.D = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B2 + 12);
    
    coeficientes_ENCODER_FLUJO_B2.A = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B2);
    coeficientes_ENCODER_FLUJO_B2.B = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B2 + 4);
    coeficientes_ENCODER_FLUJO_B2.C = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B2 + 8);
    coeficientes_ENCODER_FLUJO_B2.D = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B2 + 12);

    coeficientes_PWM_ENCODER_fake_B2.A = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B2);
    coeficientes_PWM_ENCODER_fake_B2.B = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B2 + 4);
    coeficientes_PWM_ENCODER_fake_B2.C = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B2 + 8);
    coeficientes_PWM_ENCODER_fake_B2.D = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B2 + 12);
    
    //SerialBT.println("CALIBRACION M2 CARGADA DESDE EEPROM");
    //SerialBT.println("PWM-FLUJO-B2-A: " + String(coeficientes_PWM_FLUJO_B2.A)+" PWM-FLUJO-B2-B: " + String(coeficientes_PWM_FLUJO_B2.B)+" PWM-FLUJO-B2-C: " + String(coeficientes_PWM_FLUJO_B2.C)+" PWM-FLUJO-B2-A: " + String(coeficientes_PWM_FLUJO_B2.D));
    //SerialBT.println("PWM-HALL-B2-A: " + String(coeficientes_PWM_ENCODER_fake_B2.A)+" PWM-HALL-B2-B: " + String(coeficientes_PWM_ENCODER_fake_B2.B)+" PWM-HALL-B2-C: " + String(coeficientes_PWM_ENCODER_fake_B2.C)+" PWM-HALL-B2-D: " + String(coeficientes_PWM_ENCODER_fake_B2.D));
    //SerialBT.println("HALL-FLUJO-B2-A: " + String(coeficientes_ENCODER_FLUJO_B2.A)+" HALL-FLUJO-B2-B: " + String(coeficientes_ENCODER_FLUJO_B2.B)+" HALL-FLUJO-B2-C: " + String(coeficientes_ENCODER_FLUJO_B2.C)+" HALL-FLUJO-B2-D: " + String(coeficientes_ENCODER_FLUJO_B2.D));    
 
  }
  
  if(estado_calibracion_3 ==  1){  
    coeficientes_PWM_FLUJO_B3.A = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B3);
    coeficientes_PWM_FLUJO_B3.B = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B3 + 4);
    coeficientes_PWM_FLUJO_B3.C = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B3 + 8);
    coeficientes_PWM_FLUJO_B3.D = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B3 + 12);

    coeficientes_ENCODER_FLUJO_B3.A = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B3);
    coeficientes_ENCODER_FLUJO_B3.B = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B3 + 4);
    coeficientes_ENCODER_FLUJO_B3.C = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B3 + 8);
    coeficientes_ENCODER_FLUJO_B3.D = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B3 + 12);

    coeficientes_PWM_ENCODER_fake_B3.A = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B3);
    coeficientes_PWM_ENCODER_fake_B3.B = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B3 + 4);
    coeficientes_PWM_ENCODER_fake_B3.C = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B3 + 8);
    coeficientes_PWM_ENCODER_fake_B3.D = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B3 + 12);
    
    //SerialBT.println("CALIBRACION M3 CARGADA DESDE EEPROM");
    //SerialBT.println("PWM-FLUJO-B3-A: " + String(coeficientes_PWM_FLUJO_B3.A)+" PWM-FLUJO-B3-B: " + String(coeficientes_PWM_FLUJO_B3.B)+" PWM-FLUJO-B3-C: " + String(coeficientes_PWM_FLUJO_B3.C)+" PWM-FLUJO-B3-A: " + String(coeficientes_PWM_FLUJO_B3.D));
    //SerialBT.println("HALL-FLUJO-B3-A: " + String(coeficientes_ENCODER_FLUJO_B3.A)+" HALL-FLUJO-B3-B: " + String(coeficientes_ENCODER_FLUJO_B3.B)+" HALL-FLUJO-B3-C: " + String(coeficientes_ENCODER_FLUJO_B3.C)+" HALL-FLUJO-B3-D: " + String(coeficientes_ENCODER_FLUJO_B3.D));
    //SerialBT.println("PWM-HALL-B3-A: " + String(coeficientes_PWM_ENCODER_fake_B3.A)+" PWM-HALL-B3-B: " + String(coeficientes_PWM_ENCODER_fake_B3.B)+" PWM-HALL-B3-C: " + String(coeficientes_PWM_ENCODER_fake_B3.C)+" PWM-HALL-B3-D: " + String(coeficientes_PWM_ENCODER_fake_B3.D));
  }else{E_BOMBAS = ERROR_;}
  
  if(estado_calibracion_4 ==  1){
    coeficientes_PWM_FLUJO_B4.A = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B4);
    coeficientes_PWM_FLUJO_B4.B = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B4 + 4);
    coeficientes_PWM_FLUJO_B4.C = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B4 + 8);
    coeficientes_PWM_FLUJO_B4.D = leer_float_eeprom(  direccion_coeficientes_PWM_FLUJO_B4 + 12);
  
    coeficientes_ENCODER_FLUJO_B4.A = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B4);
    coeficientes_ENCODER_FLUJO_B4.B = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B4 + 4);
    coeficientes_ENCODER_FLUJO_B4.C = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B4 + 8);
    coeficientes_ENCODER_FLUJO_B4.D = leer_float_eeprom(  direccion_coeficientes_ENCODER_FLUJO_B4 + 12);
    
    coeficientes_PWM_ENCODER_fake_B4.A = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B4);
    coeficientes_PWM_ENCODER_fake_B4.B = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B4 + 4);
    coeficientes_PWM_ENCODER_fake_B4.C = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B4 + 8);
    coeficientes_PWM_ENCODER_fake_B4.D = leer_float_eeprom(  direccion_coeficientes_PWM_ENCODER_fake_B4 + 12);
    
    //SerialBT.println("CALIBRACION M4 CARGADA DESDE EEPROM");
    //SerialBT.println("PWM-FLUJO-B4-A: " + String(coeficientes_PWM_FLUJO_B4.A)+" PWM-FLUJO-B4-B: " + String(coeficientes_PWM_FLUJO_B4.B)+" PWM-FLUJO-B4-C: " + String(coeficientes_PWM_FLUJO_B4.C)+" PWM-FLUJO-B4-A: " + String(coeficientes_PWM_FLUJO_B4.D));
    //SerialBT.println("HALL-FLUJO-B4-A: " + String(coeficientes_ENCODER_FLUJO_B4.A)+" HALL-FLUJO-B4-B: " + String(coeficientes_ENCODER_FLUJO_B4.B)+" HALL-FLUJO-B4-C: " + String(coeficientes_ENCODER_FLUJO_B4.C)+" HALL-FLUJO-B4-D: " + String(coeficientes_ENCODER_FLUJO_B4.D));
    //SerialBT.println("PWM-HALL-B4-A: " + String(coeficientes_PWM_ENCODER_fake_B4.A)+" PWM-HALL-B4-B: " + String(coeficientes_PWM_ENCODER_fake_B4.B)+" PWM-HALL-B4-C: " + String(coeficientes_PWM_ENCODER_fake_B4.C)+" PWM-HALL-B4-D: " + String(coeficientes_PWM_ENCODER_fake_B4.D));
  }else{E_BOMBAS = ERROR_;}
}


float get_flujo_total(){  
  float flujoTotal;
 
  //flujo1 = coeficientes_ENCODER_FLUJO_B1.A * (pulsosHallHZ_1*pulsosHallHZ_1*pulsosHallHZ_1) + coeficientes_ENCODER_FLUJO_B1.B * (pulsosHallHZ_1*pulsosHallHZ_1) + coeficientes_ENCODER_FLUJO_B1.C * pulsosHallHZ_1 + coeficientes_ENCODER_FLUJO_B1.D;
  //flujo2 = coeficientes_ENCODER_FLUJO_B2.A * (pulsosHallHZ_2*pulsosHallHZ_2*pulsosHallHZ_2) + coeficientes_ENCODER_FLUJO_B2.B * (pulsosHallHZ_2*pulsosHallHZ_2) + coeficientes_ENCODER_FLUJO_B2.C * pulsosHallHZ_2 + coeficientes_ENCODER_FLUJO_B2.D;
  //flujo3 = coeficientes_ENCODER_FLUJO_B3.A * (pulsosHallHZ_3*pulsosHallHZ_3*pulsosHallHZ_3) + coeficientes_ENCODER_FLUJO_B3.B * (pulsosHallHZ_3*pulsosHallHZ_3) + coeficientes_ENCODER_FLUJO_B3.C * pulsosHallHZ_3 + coeficientes_ENCODER_FLUJO_B3.D;
  //flujo4 = coeficientes_ENCODER_FLUJO_B4.A * (pulsosHallHZ_4*pulsosHallHZ_4*pulsosHallHZ_4) + coeficientes_ENCODER_FLUJO_B4.B * (pulsosHallHZ_4*pulsosHallHZ_4) + coeficientes_ENCODER_FLUJO_B4.C * pulsosHallHZ_4 + coeficientes_ENCODER_FLUJO_B4.D;
  flujo1 = Input_B1;
  flujo2 = Input_B2;
  flujo3 = Input_B3;
  flujo4 = Input_B4;
  
  if(pulsosHallHZ_1<1)
    flujo1=0;
  if(pulsosHallHZ_2<1)
    flujo2=0;
  if(pulsosHallHZ_3<1)
    flujo3=0;
  if(pulsosHallHZ_4<1)
    flujo4=0;
    
  if(flujo1>1800)
    flujo1=1800;
  if(flujo2>1800)
    flujo2=1800;
  if(flujo3>1800)
    flujo3=1800;
  if(flujo4>1800)
    flujo4=1800;

  flujoTotal=flujo1+flujo2+flujo3+flujo4;
  //Serial.println(" flujo_1: " + String(flujo1) + " flujo_2: "+ String(flujo2) + " flujo_3: "+ String(flujo3) + " flujo_4: " + String(flujo4));

  if(flujoTotal>10){
    return flujoTotal;
  }else{
    return 0;
  }
}


void ARDUINO_ISR_ATTR onTimerAlarm(){
  // ====================ZONA CRITICA DE VARIABLES VOLATILES====================
  portENTER_CRITICAL_ISR(&timerMux);

  cPulsosTimerD1=contPulsosD1;
  acumTAltoTimerD1=acumTAltoD1;
  acumTBajoTimerD1=acumTBajoD1;
  contPulsosD1=0;
  acumTAltoD1=0;
  acumTBajoD1=0;
  tiempoAnteriorD1=0;

  cPulsosTimerD2=contPulsosD2;
  acumTAltoTimerD2=acumTAltoD2;
  acumTBajoTimerD2=acumTBajoD2;
  contPulsosD2=0;
  acumTAltoD2=0;
  acumTBajoD2=0;
  tiempoAnteriorD2=0;

  cPulsosTimerD3=contPulsosD3;
  acumTAltoTimerD3=acumTAltoD3;
  acumTBajoTimerD3=acumTBajoD3;
  contPulsosD3=0;
  acumTAltoD3=0;
  acumTBajoD3=0;
  tiempoAnteriorD3=0;

  cPulsosTimerD4=contPulsosD4;
  acumTAltoTimerD4=acumTAltoD4;
  acumTBajoTimerD4=acumTBajoD4;
  contPulsosD4=0;
  acumTAltoD4=0;
  acumTBajoD4=0;
  tiempoAnteriorD4=0;

  cPulsosTimerHall1=contPulsosHall1;
  contPulsosHall1=0;
  cPulsosTimerHall2=contPulsosHall2;
  contPulsosHall2=0;
  cPulsosTimerHall3=contPulsosHall3;
  contPulsosHall3=0;
  cPulsosTimerHall4=contPulsosHall4;
  contPulsosHall4=0;
  


  portEXIT_CRITICAL_ISR(&timerMux);
  // ====================ZONA CRITICA DE VARIABLES VOLATILES====================
  // Semaforo del contador del timer
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // Se puede utilizar digitalWrite/Read aquí
}


void ARDUINO_ISR_ATTR isrGetDuty1(){  
  tiempoActualD1=timerRead(timer);
  if(tiempoActualD1>tiempoAnteriorD1){
    if(digitalRead(PIN_INPUT_DUTYCYCLE_1)==1){
      contPulsosD1++;
      if(tiempoAnteriorD1!=0){
        tiempoBajoD1=tiempoActualD1-tiempoAnteriorD1;
        acumTBajoD1+=tiempoBajoD1;
      }        
    }
    else if(digitalRead(PIN_INPUT_DUTYCYCLE_1)==0){
      tiempoAltoD1=tiempoActualD1-tiempoAnteriorD1;
      acumTAltoD1+=tiempoAltoD1;
    }    
    tiempoAnteriorD1=tiempoActualD1;
  }
  if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_1,digitalRead(PIN_INPUT_DUTYCYCLE_1));}
}

void ARDUINO_ISR_ATTR isrGetDuty2(){  
  tiempoActualD2=timerRead(timer);
  if(tiempoActualD2>tiempoAnteriorD2){
    if(digitalRead(PIN_INPUT_DUTYCYCLE_2)==1){
      contPulsosD2++;
      if(tiempoAnteriorD2!=0){
        tiempoBajoD2=tiempoActualD2-tiempoAnteriorD2;
        acumTBajoD2+=tiempoBajoD2;
      }        
    }
    else if(digitalRead(PIN_INPUT_DUTYCYCLE_2)==0){
      tiempoAltoD2=tiempoActualD2-tiempoAnteriorD2;
      acumTAltoD2+=tiempoAltoD2;
    }    
    tiempoAnteriorD2=tiempoActualD2;
  }
  
  if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_2,digitalRead(PIN_INPUT_DUTYCYCLE_2));}

}

void ARDUINO_ISR_ATTR isrGetDuty3(){  
  tiempoActualD3=timerRead(timer);
  if(tiempoActualD3>tiempoAnteriorD3){
    if(digitalRead(PIN_INPUT_DUTYCYCLE_3)==1){
      contPulsosD3++;
      if(tiempoAnteriorD3!=0){
        tiempoBajoD3=tiempoActualD3-tiempoAnteriorD3;
        acumTBajoD3+=tiempoBajoD3;
      }        
    }
    else if(digitalRead(PIN_INPUT_DUTYCYCLE_3)==0){
      tiempoAltoD3=tiempoActualD3-tiempoAnteriorD3;
      acumTAltoD3+=tiempoAltoD3;
    }    
    tiempoAnteriorD3=tiempoActualD3;
  }

  if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_3,digitalRead(PIN_INPUT_DUTYCYCLE_3));}

}

void ARDUINO_ISR_ATTR isrGetDuty4(){  
  tiempoActualD4=timerRead(timer);
  if(tiempoActualD4>tiempoAnteriorD4){
    if(digitalRead(PIN_INPUT_DUTYCYCLE_4)==1){
      contPulsosD4++;
      if(tiempoAnteriorD4!=0){
        tiempoBajoD4=tiempoActualD4-tiempoAnteriorD4;
        acumTBajoD4+=tiempoBajoD4;
      }        
    }
    else if(digitalRead(PIN_INPUT_DUTYCYCLE_4)==0){
      tiempoAltoD4=tiempoActualD4-tiempoAnteriorD4;
      acumTAltoD4+=tiempoAltoD4;
    }    
    tiempoAnteriorD4=tiempoActualD4;
  }

  if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_4,digitalRead(PIN_INPUT_DUTYCYCLE_4));}

}

                                                                                                        //// MODIFICADO
void ARDUINO_ISR_ATTR isrContHall1(){
  contPulsosHall1++;
  if(pasar_pulsos){
    digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,digitalRead(PIN_INPUT_HALL_1));
  }
}

void ARDUINO_ISR_ATTR isrContHall2(){
  contPulsosHall2++;
  if(pasar_pulsos){
    digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,digitalRead(PIN_INPUT_HALL_2));
  }
}

void ARDUINO_ISR_ATTR isrContHall3(){
  contPulsosHall3++;
  if(pasar_pulsos){
    digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,digitalRead(PIN_INPUT_HALL_3));
  }
}

void ARDUINO_ISR_ATTR isrContHall4(){
  contPulsosHall4++;
  if(pasar_pulsos){
    digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,digitalRead(PIN_INPUT_HALL_4));
  }
}


void setupTimerAlarm(){
  // ===============================CONFIGURACION DEL TIMER==========================================
  // Se crea un objeto semaforo que informará de la activación del timer
  timerSemaphore = xSemaphoreCreateBinary();
  // Primer parametro:numero de timer(0-3),
  // segundo parametro: divisor de la frecuencia base(en este caso la frecuencia de la esp32 es de 80MHz).
  // Al dividirlo entre 80 se obtiene 1MHz, lo que significa que se configura el timer para que su contador se incremente cada microsegundo
  // El tercer parametro establece si el contador ira en incremento(true) o en decremento(false)
  timer = timerBegin(3, 80, true);

  // El primer parametro es un objeto tipo timer
  // El segundo es la direccion de la funcion donde se ejecutará el codigo una vez que el timer llegue a su cuenta maxima
  // Solo la opcion "false" esta disponible
  timerAttachInterrupt(timer, &onTimerAlarm, false);

  // Primer parametro: estructura timer
  // Segundo parametro: el numero al cual el contador del timer generará la interrupción del timer
  // true si se quiere que se reinicie la cuenta del contador del timer, en caso contrario "false"
  timerAlarmWrite(timer, 10000, true);                                                                                          ////////// MODIFICADO

  // Habilta el timer configurado
  timerAlarmEnable(timer);
  // ===============================CONFIGURACION DEL TIMER==========================================
}


void setupPinInterrupcionDuty1(){
  pinMode(PIN_INPUT_DUTYCYCLE_1,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_DUTYCYCLE_1),isrGetDuty1,CHANGE);
}

void setupPinInterrupcionDuty2(){
  pinMode(PIN_INPUT_DUTYCYCLE_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_DUTYCYCLE_2),isrGetDuty2,CHANGE);
}

void setupPinInterrupcionDuty3(){
  pinMode(PIN_INPUT_DUTYCYCLE_3,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_DUTYCYCLE_3),isrGetDuty3,CHANGE);
}

void setupPinInterrupcionDuty4(){
  pinMode(PIN_INPUT_DUTYCYCLE_4,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_DUTYCYCLE_4),isrGetDuty4,CHANGE);
}

                                                                                        /// CAMBIADO
void setupPinInterrupcionHall1(){
  pinMode(PIN_INPUT_HALL_1,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_HALL_1),isrContHall1,CHANGE);
}

void setupPinInterrupcionHall2(){
  pinMode(PIN_INPUT_HALL_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_HALL_2),isrContHall2,CHANGE);
}

void setupPinInterrupcionHall3(){
  pinMode(PIN_INPUT_HALL_3,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_HALL_3),isrContHall3,CHANGE);
}

void setupPinInterrupcionHall4(){
  pinMode(PIN_INPUT_HALL_4,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_HALL_4),isrContHall4,CHANGE);
}


void setupPinesInterrupcionHall(){
  setupPinInterrupcionHall1();
  setupPinInterrupcionHall2();
  setupPinInterrupcionHall3();
  setupPinInterrupcionHall4();
}

void setupPinesInterrupcionDuty(){
  setupPinInterrupcionDuty1();
  setupPinInterrupcionDuty2();
  setupPinInterrupcionDuty3();
  setupPinInterrupcionDuty4();
}


void setupGeneradorDeFrecuencia1(){
  pinMode(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,OUTPUT);
  digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,0);
  //Primer parametro: el canal seleccionado(pueden ser del 0-16 en la esp32)
  //Segundo parametro: la frecuencia que tendra la señal
  //Tercer parametro: los bits de resolucion que tendra la señal
  ledcSetup(CANAL_FREC_VAR_GEN_1,FRECUENCIA_INICIAL_FV_1,RESOLUCION_CANAL_FV_1);//Se configura el pwm del canal 0 a 10 hz con una resolucion de 10 bits
  //Primer parametro: pin que sera la salida de la señal
  //Segundo:canal relacionado para la señal generada
  //ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,CANAL_FREC_VAR_GEN_1);//Se asocia el puerto 25 al pwm del canal 0
  //Primer parametro: canal seleccionado de la señal
  //Segundo: frecuencia a la que trabajara la señal
  ledcWriteTone(CANAL_FREC_VAR_GEN_1,FRECUENCIA_INICIAL_FV_1);//Se asocia la frecencia del canal 0 con un dutycycle de 50%
}

void setupGeneradorDeFrecuencia2(){
  pinMode(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,OUTPUT);
  digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,0);
  //Primer parametro: el canal seleccionado(pueden ser del 0-16 en la esp32)
  //Segundo parametro: la frecuencia que tendra la señal
  //Tercer parametro: los bits de resolucion que tendra la señal
  ledcSetup(CANAL_FREC_VAR_GEN_2,FRECUENCIA_INICIAL_FV_2,RESOLUCION_CANAL_FV_2);//Se configura el pwm del canal 0 a 10 hz con una resolucion de 10 bits
  //Primer parametro: pin que sera la salida de la señal
  //Segundo:canal relacionado para la señal generada
  //ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,CANAL_FREC_VAR_GEN_2);//Se asocia el puerto 25 al pwm del canal 0
  //Primer parametro: canal seleccionado de la señal
  //Segundo: frecuencia a la que trabajara la señal
  ledcWriteTone(CANAL_FREC_VAR_GEN_2,FRECUENCIA_INICIAL_FV_2);//Se asocia la frecencia del canal 0 con un dutycycle de 50%
}

void setupGeneradorDeFrecuencia3(){
  pinMode(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,OUTPUT);
  digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,0);
  //Primer parametro: el canal seleccionado(pueden ser del 0-16 en la esp32)
  //Segundo parametro: la frecuencia que tendra la señal
  //Tercer parametro: los bits de resolucion que tendra la señal
  ledcSetup(CANAL_FREC_VAR_GEN_3,FRECUENCIA_INICIAL_FV_3,RESOLUCION_CANAL_FV_3);//Se configura el pwm del canal 0 a 10 hz con una resolucion de 10 bits
  //Primer parametro: pin que sera la salida de la señal
  //Segundo:canal relacionado para la señal generada
  //ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,CANAL_FREC_VAR_GEN_3);//Se asocia el puerto 25 al pwm del canal 0
  //Primer parametro: canal seleccionado de la señal
  //Segundo: frecuencia a la que trabajara la señal
  ledcWriteTone(CANAL_FREC_VAR_GEN_3,FRECUENCIA_INICIAL_FV_3);//Se asocia la frecencia del canal 0 con un dutycycle de 50%
}

void setupGeneradorDeFrecuencia4(){
  pinMode(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,OUTPUT);
  digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,0);
  //Primer parametro: el canal seleccionado(pueden ser del 0-16 en la esp32)
  //Segundo parametro: la frecuencia que tendra la señal
  //Tercer parametro: los bits de resolucion que tendra la señal
  ledcSetup(CANAL_FREC_VAR_GEN_4,FRECUENCIA_INICIAL_FV_4,RESOLUCION_CANAL_FV_4);//Se configura el pwm del canal 0 a 10 hz con una resolucion de 10 bits
  //Primer parametro: pin que sera la salida de la señal
  //Segundo:canal relacionado para la señal generada
  //ledcAttachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,CANAL_FREC_VAR_GEN_4);//Se asocia el puerto 25 al pwm del canal 0
  //Primer parametro: canal seleccionado de la señal
  //Segundo: frecuencia a la que trabajara la señal
  ledcWriteTone(CANAL_FREC_VAR_GEN_4,FRECUENCIA_INICIAL_FV_4);//Se asocia la frecencia del canal 0 con un dutycycle de 50%
}


void loop2(void *pvParameters){
  for (;;){
    computePID(); 
    check_sistemas();
    /*
    if (SerialBT.available()) {
    
      String cadenaComando = SerialBT.readStringUntil('\n');
      String S_kp_B1 = separador(cadenaComando,',', 0);
      String S_ki_B1 = separador(cadenaComando,',', 1);
      String S_kd_B1 = separador(cadenaComando,',', 2);
      
      kp_B1=S_kp_B1.toFloat();
      ki_B1=S_ki_B1.toFloat();
      kd_B1=S_kd_B1.toFloat();
  
      SerialBT.println("kp_B1: " + String(kp_B1,4) + " ki_B1: " + String(ki_B1,4) + " kd_B1: " + String(kd_B1,4));
      
    }*/
  }
}


void setupGeneradoresDeFrecuencia(){
  setupGeneradorDeFrecuencia1();
  setupGeneradorDeFrecuencia2();
  setupGeneradorDeFrecuencia3();
  setupGeneradorDeFrecuencia4();
}


void setupPwmMotor1(){
  pinMode(PIN_OUTPUT_PWM_MOTOR_1,OUTPUT);
  ledcSetup(CANAL_PWM_MOTOR_1,FRECUENCIA_PWM_MOTOR_1,RESOLUCION_CANAL_PWM_MOTOR_1);
  //ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_1,CANAL_PWM_MOTOR_1);
  digitalWrite(PIN_OUTPUT_PWM_MOTOR_1,0);
}

void setupPwmMotor2(){
  pinMode(PIN_OUTPUT_PWM_MOTOR_2,OUTPUT);
  ledcSetup(CANAL_PWM_MOTOR_2,FRECUENCIA_PWM_MOTOR_2,RESOLUCION_CANAL_PWM_MOTOR_2);
  //ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_2,CANAL_PWM_MOTOR_2);
  digitalWrite(PIN_OUTPUT_PWM_MOTOR_2,0);
}

void setupPwmMotor3(){
  pinMode(PIN_OUTPUT_PWM_MOTOR_3,OUTPUT);
  digitalWrite(PIN_OUTPUT_PWM_MOTOR_3,0);
  ledcSetup(CANAL_PWM_MOTOR_3,FRECUENCIA_PWM_MOTOR_3,RESOLUCION_CANAL_PWM_MOTOR_3);
  //ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_3,CANAL_PWM_MOTOR_3);
}

void setupPwmMotor4(){
  pinMode(PIN_OUTPUT_PWM_MOTOR_4,OUTPUT);
  digitalWrite(PIN_OUTPUT_PWM_MOTOR_4,0);
  ledcSetup(CANAL_PWM_MOTOR_4,FRECUENCIA_PWM_MOTOR_4,RESOLUCION_CANAL_PWM_MOTOR_4);
  //ledcAttachPin(PIN_OUTPUT_PWM_MOTOR_4,CANAL_PWM_MOTOR_4);
}

void setupPwmMotores(){
  setupPwmMotor1();
  setupPwmMotor2();
  setupPwmMotor3();
  setupPwmMotor4();
}


void setup() { 
  delay(100);
  Serial.begin(500000);  
  Serial.println("Serial ok");
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM ok");
  setupTimerAlarm();
  Serial.println("Timer ok");
  setupPinesInterrupcionDuty();
  Serial.println("Interrupciones duty ok");
  setupPinesInterrupcionHall(); 
  Serial.println("Interrupciones hall ok");
  setupGeneradoresDeFrecuencia(); 
  Serial.println("Generadores de frecuencia ok");
  setupPwmMotores();  
  Serial.println("Pwm de motores ok");
  xTaskCreatePinnedToCore(loop2, "Task1", 5000, NULL, 0, &Task1, 0);
  Serial.println("Segundo nucleo ok");  
  setup_i2c();
  Serial.println("i2c ok"); 
  cargar_calibracion_bombas();

  //SerialBT.begin("ESP32test");
  delay(500);

}


void genFrecApartirDeDuty1(float duty){
  frecVarGenerada1 =coeficientes_PWM_ENCODER_fake_B1.C * duty + coeficientes_PWM_ENCODER_fake_B1.D; // coeficientes_PWM_ENCODER_fake_B1.B * (duty*duty) + 
  if(frecVarGenerada1<=10){frecVarGenerada1=10;}  
  if(duty>80){frecVarGenerada1=frecVarGenerada1 * 0.9;}
  ledcWriteTone(CANAL_FREC_VAR_GEN_1,frecVarGenerada1);
}

void genFrecApartirDeDuty2(float duty){
  frecVarGenerada2 = coeficientes_PWM_ENCODER_fake_B2.C * duty + coeficientes_PWM_ENCODER_fake_B2.D; //coeficientes_PWM_ENCODER_fake_B2.A * (duty*duty*duty) + coeficientes_PWM_ENCODER_fake_B2.B * (duty*duty) + 
  if(frecVarGenerada2<=10){frecVarGenerada2=10;}  
  if(duty>80){frecVarGenerada2=frecVarGenerada2 * 0.9;}
  ledcWriteTone(CANAL_FREC_VAR_GEN_2,frecVarGenerada2);
}

void genFrecApartirDeDuty3(float duty){
  frecVarGenerada3 = coeficientes_PWM_ENCODER_fake_B3.C * duty + coeficientes_PWM_ENCODER_fake_B3.D; //coeficientes_PWM_ENCODER_fake_B3.A * (duty*duty*duty) + coeficientes_PWM_ENCODER_fake_B3.B * (duty*duty) + 
  if(frecVarGenerada3<=10){frecVarGenerada3=10;}  
  if(duty>80){frecVarGenerada3=frecVarGenerada3 * 0.9;}
  ledcWriteTone(CANAL_FREC_VAR_GEN_3,frecVarGenerada3);
}

void genFrecApartirDeDuty4(float duty){
  frecVarGenerada4 = coeficientes_PWM_ENCODER_fake_B4.C * duty + coeficientes_PWM_ENCODER_fake_B4.D; // coeficientes_PWM_ENCODER_fake_B4.A * (duty*duty*duty) + coeficientes_PWM_ENCODER_fake_B4.B * (duty*duty) + 
  if(frecVarGenerada4<=10){frecVarGenerada4=10;}  
  if(duty>80){frecVarGenerada4=frecVarGenerada4 * 0.9;}
  ledcWriteTone(CANAL_FREC_VAR_GEN_4,frecVarGenerada4);
}


void generarPwmMotor1(int dutycycleMotor){
  int duty=int(dutycycleMotor);
  if(!isnan(duty)){
    if (duty < MIN_DUTY_WORK){duty = MIN_DUTY_WORK;}
    if (duty > MAX_DUTY_WORK){duty = MAX_DUTY_WORK;}
    ledcWrite(CANAL_PWM_MOTOR_1,duty);
    //Serial.print(" motor 1 al: "+String(duty));
  }else{ledcWrite(CANAL_PWM_MOTOR_1,0);}
}

void generarPwmMotor2(int dutycycleMotor){
  int duty=int(dutycycleMotor);
  if(!isnan(duty)){
    if (duty < MIN_DUTY_WORK){duty = MIN_DUTY_WORK;}
    if (duty > MAX_DUTY_WORK){duty = MAX_DUTY_WORK;}
    ledcWrite(CANAL_PWM_MOTOR_2,duty);
    //Serial.print(" motor 2 al: "+String(duty));
  }else{ledcWrite(CANAL_PWM_MOTOR_2,0);};
}

void generarPwmMotor3(int dutycycleMotor){
  int duty=int(dutycycleMotor);
  if(!isnan(duty)){
    if (duty < MIN_DUTY_WORK){duty = MIN_DUTY_WORK;}
    if (duty > MAX_DUTY_WORK){duty = MAX_DUTY_WORK;}
    ledcWrite(CANAL_PWM_MOTOR_3,duty);
    //Serial.print(" motor 3 al: "+String(duty));
  }else{ledcWrite(CANAL_PWM_MOTOR_3,0);}
}

void generarPwmMotor4(int dutycycleMotor){
  int duty=int(dutycycleMotor);
  if(!isnan(duty)){
    if (duty < MIN_DUTY_WORK){duty = MIN_DUTY_WORK;}
    if (duty > MAX_DUTY_WORK){duty = MAX_DUTY_WORK;}
    ledcWrite(CANAL_PWM_MOTOR_4,duty);
    //Serial.print(" motor 4 al: "+String(duty));
  }else{ledcWrite(CANAL_PWM_MOTOR_4,0);}
}


bool guardar_float_eeprom(float valor_float, int direccion){
  if (!isnan(valor_float)){
    byte * dato_byte = (byte *) &valor_float;
    for(int i=0; i < 4; i++){
      EEPROM.write(direccion + i, dato_byte[i]);
    }
    EEPROM.commit();
    return true; 
  }else{
    return false;
  }
}

float leer_float_eeprom(int direccion){
  byte dato_array_float[4];
  float dato_leido;
  for(int i=0; i < 4; i++){
    dato_array_float[i] = EEPROM.read(direccion + i);
  }
  dato_leido = *((float*)dato_array_float);
  return dato_leido;
}


void calcularDutyCycle(){
  // Cuando la interrupcion del timer se ejecute el semaforo será igual a "pdTRUE"
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // =========================ZONA CRITICA DE RESPALDO DE USO VOLATILES=========================
    portENTER_CRITICAL(&timerMux);

    cPulsosTempD1=cPulsosTimerD1;
    acumTAltoTempD1=acumTAltoTimerD1;
    acumTBajoTempD1=acumTBajoTimerD1;

    cPulsosTempD2=cPulsosTimerD2;
    acumTAltoTempD2=acumTAltoTimerD2;
    acumTBajoTempD2=acumTBajoTimerD2;

    cPulsosTempD3=cPulsosTimerD3;
    acumTAltoTempD3=acumTAltoTimerD3;
    acumTBajoTempD3=acumTBajoTimerD3;

    cPulsosTempD4=cPulsosTimerD4;
    acumTAltoTempD4=acumTAltoTimerD4;
    acumTBajoTempD4=acumTBajoTimerD4;
                                                                                                                            ///// MODIFICADO
    pulsosHallPorMin1=cPulsosTimerHall1 * 50; // pulsos en 10 000 microsegundos
    pulsosHallPorMin2=cPulsosTimerHall2 * 50;
    pulsosHallPorMin3=cPulsosTimerHall3 * 50;
    pulsosHallPorMin4=cPulsosTimerHall4 * 50;
    
    portEXIT_CRITICAL(&timerMux);
    // =========================ZONA CRITICA DE RESPALDO DE USO VOLATILES=========================
    
    if(cPulsosTempD1>10){
      dutycycle1=(acumTAltoTempD1*100.0)/(acumTAltoTempD1+acumTBajoTempD1);
    }else{
      dutycycle1 =0;
      error_B1 = 0;
      cumError_B1 = 0; 
      rateError_B1 = 0;
      if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_1,0);}
    } 

    if(cPulsosTempD2>10){      
      dutycycle2=(acumTAltoTempD2*100.0)/(acumTAltoTempD2+acumTBajoTempD2); 
    }else{
      dutycycle2 = 0;
      error_B2 = 0;
      cumError_B2 = 0; 
      rateError_B2 = 0;
      if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_2,0);}
    }

    if(cPulsosTempD3>10){      
      dutycycle3=(acumTAltoTempD3*100.0)/(acumTAltoTempD3+acumTBajoTempD3); 
    }else{
      dutycycle3 = 0;
      error_B3 = 0;
      cumError_B3 = 0; 
      rateError_B3 = 0;
      if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_3,0);}
    }

    if(cPulsosTempD4>10){      
      dutycycle4=(acumTAltoTempD4*100.0)/(acumTAltoTempD4+acumTBajoTempD4); 
    }else{
      dutycycle4 = 0;
      error_B4 = 0;
      cumError_B4 = 0; 
      rateError_B4 = 0;
      if(pasar_pulsos){digitalWrite(PIN_OUTPUT_PWM_MOTOR_4,0);}
    }

    
    pulsosHallHZ_1 = EMA_ALPHA_PULSO_IN * pulsosHallPorMin1 + (1 - EMA_ALPHA_PULSO_IN) * pulsosHallHZ_1;
    pulsosHallHZ_2 = EMA_ALPHA_PULSO_IN * pulsosHallPorMin2 + (1 - EMA_ALPHA_PULSO_IN) * pulsosHallHZ_2;
    pulsosHallHZ_3 = EMA_ALPHA_PULSO_IN * pulsosHallPorMin3 + (1 - EMA_ALPHA_PULSO_IN) * pulsosHallHZ_3;
    pulsosHallHZ_4 = EMA_ALPHA_PULSO_IN * pulsosHallPorMin4 + (1 - EMA_ALPHA_PULSO_IN) * pulsosHallHZ_4;

    dutycycle1_fill = EMA_ALPHA_DUTY_IN * dutycycle1 + (1 - EMA_ALPHA_DUTY_IN) * dutycycle1_fill;
    dutycycle2_fill = EMA_ALPHA_DUTY_IN * dutycycle2 + (1 - EMA_ALPHA_DUTY_IN) * dutycycle2_fill;
    dutycycle3_fill = EMA_ALPHA_DUTY_IN * dutycycle3 + (1 - EMA_ALPHA_DUTY_IN) * dutycycle3_fill;
    dutycycle4_fill = EMA_ALPHA_DUTY_IN * dutycycle4 + (1 - EMA_ALPHA_DUTY_IN) * dutycycle4_fill;

    if(!pasar_pulsos){
      genFrecApartirDeDuty1(dutycycle1_fill);
      genFrecApartirDeDuty2(dutycycle2_fill);
      genFrecApartirDeDuty3(dutycycle3_fill);
      genFrecApartirDeDuty4(dutycycle4_fill);
    }
    // calculo de setpoint en mmlitros

    Input_B1 = coeficientes_ENCODER_FLUJO_B1.C * pulsosHallHZ_1 + coeficientes_ENCODER_FLUJO_B1.D; //coeficientes_ENCODER_FLUJO_B1.A * (pulsosHallHZ_1*pulsosHallHZ_1*pulsosHallHZ_1) + coeficientes_ENCODER_FLUJO_B1.B * (pulsosHallHZ_1*pulsosHallHZ_1) + coeficientes_ENCODER_FLUJO_B1.C * pulsosHallHZ_1 + coeficientes_ENCODER_FLUJO_B1.D;
    Input_B2 = coeficientes_ENCODER_FLUJO_B2.C * pulsosHallHZ_2 + coeficientes_ENCODER_FLUJO_B2.D; //coeficientes_ENCODER_FLUJO_B2.A * (pulsosHallHZ_2*pulsosHallHZ_2*pulsosHallHZ_2) + coeficientes_ENCODER_FLUJO_B2.B * (pulsosHallHZ_2*pulsosHallHZ_2) + coeficientes_ENCODER_FLUJO_B2.C * pulsosHallHZ_2 + coeficientes_ENCODER_FLUJO_B2.D;
    Input_B3 = coeficientes_ENCODER_FLUJO_B3.C * pulsosHallHZ_3 + coeficientes_ENCODER_FLUJO_B3.D; //coeficientes_ENCODER_FLUJO_B3.A * (pulsosHallHZ_3*pulsosHallHZ_3*pulsosHallHZ_3) + coeficientes_ENCODER_FLUJO_B3.B * (pulsosHallHZ_3*pulsosHallHZ_3) + coeficientes_ENCODER_FLUJO_B3.C * pulsosHallHZ_3 + coeficientes_ENCODER_FLUJO_B3.D;
    Input_B4 = coeficientes_ENCODER_FLUJO_B4.C * pulsosHallHZ_4 + coeficientes_ENCODER_FLUJO_B4.D; //coeficientes_ENCODER_FLUJO_B4.A * (pulsosHallHZ_4*pulsosHallHZ_4*pulsosHallHZ_4) + coeficientes_ENCODER_FLUJO_B4.B * (pulsosHallHZ_4*pulsosHallHZ_4) + coeficientes_ENCODER_FLUJO_B4.C * pulsosHallHZ_4 + coeficientes_ENCODER_FLUJO_B4.D;

    if(pulsosHallHZ_1 < 1){Input_B1 = 0;}
    if(pulsosHallHZ_2 < 1){Input_B2 = 0;}
    if(pulsosHallHZ_3 < 1){Input_B3 = 0;}
    if(pulsosHallHZ_4 < 1){Input_B4 = 0;}

    if(Input_B1 > 2000){Input_B1 = 2000;}
    if(Input_B2 > 2000){Input_B2 = 2000;}
    if(Input_B3 > 2000){Input_B3 = 2000;}
    if(Input_B4 > 2000){Input_B4 = 2000;}

/*
      
    
    Serial.print("dutycycle 1:  ");
    Serial.print(dutycycle1);
    Serial.print("  dutycycle 2:  ");
    Serial.print(dutycycle2);
    Serial.print("  dutycycle 3:  ");
    Serial.print(dutycycle3);
    Serial.print("  dutycycle 4:  ");
    Serial.print(dutycycle4);
    
    Serial.print("   Hall1: ");
    Serial.print(pulsosHallHZ_1);
    Serial.print("  Hall2: ");
    Serial.print(pulsosHallHZ_2);
    Serial.print("  Hall3: ");
    Serial.print(pulsosHallHZ_3);
    Serial.print("  Hall4: ");
    Serial.println(pulsosHallHZ_4);

    Serial.print("   FRECUENCIA generado H1:  ");
    Serial.print(frecVarGenerada1);
    Serial.print("  H2:  ");
    Serial.print(frecVarGenerada2);
    Serial.print("  H3:  ");
    Serial.print(frecVarGenerada3);
    Serial.print("  H4:  ");
    Serial.print(frecVarGenerada4);
    
    Serial.print("   Hall1: ");
    Serial.print(cPulsosTimerHall1); //hz del sensor hall
    Serial.print("  Hall2: ");
    Serial.print(cPulsosTimerHall2);
    Serial.print("  Hall3: ");
    Serial.print(cPulsosTimerHall3);
    Serial.print("  Hall4: ");
    Serial.println(cPulsosTimerHall4);
    
        
         * 
    Serial.print(dutycycle1_fill); // flujo deseado
    Serial.print(",");
    Serial.print(Setpoint_B1_fill); // flujo deseado
    Serial.print(",");
    Serial.println(Input_B1); // flujo real
    Serial.print(",");
    Serial.print(Output_B1);
    
    Serial.print(dutycycle1_fill);
    Serial.print(",");
    Serial.print(dutycycle1_fill);
    Serial.print(",");
    Serial.println(dutycycle1_fill);

    
    Serial.print("dutycycle 1:  ");
    Serial.print(dutycycle1);
    Serial.print("  dutycycle 2:  ");
    Serial.print(dutycycle2);
    Serial.print("  dutycycle 3:  ");
    Serial.print(dutycycle3);
    Serial.print("  dutycycle 4:  ");
    Serial.print(dutycycle4);  
    
    Serial.print("  PWM generada M1:  ");
    Serial.print(pwm_generado1);
    Serial.print("  M2:  ");
    Serial.print(pwm_generado2);
    Serial.print("  M3:  ");
    Serial.print(pwm_generado3);
    Serial.print("  M4:  ");
    Serial.print(pwm_generado4);

    Serial.print("   FRECUENCIA generado H1:  ");
    Serial.print(frecVarGenerada1);
    Serial.print("  H2:  ");
    Serial.print(frecVarGenerada2);
    Serial.print("  H3:  ");
    Serial.print(frecVarGenerada3);
    Serial.print("  H4:  ");
    Serial.print(frecVarGenerada4);
    
    Serial.print("   Hall1: ");
    Serial.print(cPulsosTimerHall1); //hz del sensor hall
    Serial.print("  Hall2: ");
    Serial.print(cPulsosTimerHall2);
    Serial.print("  Hall3: ");
    Serial.print(cPulsosTimerHall3);
    Serial.print("  Hall4: ");
    Serial.println(cPulsosTimerHall4);
    */
  }
}

void check_sistemas(){
  if((millis() - ultima_vez_12c_exitoso) > 2000){
    count_i2c_fail ++;
    if (count_i2c_fail > 10){
      if(estado_drone == TIERRA || estado_drone == AIRE){
        pasar_pulsos = true;
        //SerialBT.println("EEROR PERDIDA DE I2C");
        if(!pasar_pulsos_anterior){
          //SerialBT.println("CONFIGURACION PASAR PULSOS");
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3);
          ledcDetachPin(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4);
            
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_1);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_2);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_3);
          ledcDetachPin(PIN_OUTPUT_PWM_MOTOR_4);
  
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_1,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_2,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_3,0);
          digitalWrite(PIN_OUTPUT_FRECUENCIA_VARIABLE_GENERADA_4,0);
            
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_1,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_2,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_3,0);
          digitalWrite(PIN_OUTPUT_PWM_MOTOR_4,0);
            
          EMA_ALPHA_PULSO_IN = 0.1;
          EMA_ALPHA_DUTY_IN = 0.4;
          E_BOMBAS = OK_;
          errores_B1 = 0;
          errores_B2 = 0;
          errores_B3 = 0;
          errores_B4 = 0;
        }
        pasar_pulsos_anterior = pasar_pulsos;
      }
    }
  }else{
    count_i2c_fail = 0;
  }

}

void loop() {   
  calcularDutyCycle();
}
