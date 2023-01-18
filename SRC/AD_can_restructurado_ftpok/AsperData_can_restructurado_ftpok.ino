#include <TFmini.h>
#include <SD.h>
#include <WiFi.h>
#include "FtpServer.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <CAN.h> // usar libreria modificada

const char* ssid     = "AsperDrone_1";     // NOMBRE DE RED
const char* password = "abcdefghi";        // CONTRASEÑA
const char* username_server = "esp32";      // NOMBRE SERVIDOR
const char* password_server = "esp32";      // CONTRASEÑA SERVIDOR

TaskHandle_t Task1;

// general Y MOTORES
#define INICIANDO                 5
#define ERROR_                    0
#define OK_                       1
// gps
#define BUSCANDO                  2
// dron
#define TIERRA                    0
#define AIRE                      1
#define CALIBRACION               2
#define GUARDAR_CALIBRACION       3
// log
#define LISTO_PARA_GUARDAR        1
#define GUARDANDO                 2
#define PAUSADO                   3
// mapa pwm
#define MAPA_PREVIO_CARGADO       2
#define NO_CARGADO                0
#define MAPA_CARGADO_DESDE_APP    3
//SOCKET
#define SOCKET_CONECTADO          1
#define SOCKET_LISTO              2
// FTP
#define FTP_CONECTADO             1
#define FTP_LISTO                 2
//WIFI
#define RED_LISTA                 1
#define CONECTADO                 2



// ==================================== ESTADOS DEL SISTEMA ===================================================

int E_DRONE = TIERRA;
int E_GPS = INICIANDO;
int E_TF = INICIANDO;
int E_SOCKET = INICIANDO;
int E_FTP = INICIANDO;
int E_WIFI = INICIANDO;
int E_MAPA = NO_CARGADO;
int E_SD = INICIANDO;
int E_LOG = INICIANDO;
int E_ESP_NOW = INICIANDO;
int E_BOMBAS = INICIANDO;
int E_I2C = INICIANDO;


// ==================================== VARIABLES TIEMPO DE EJECUCION ===================================================

#define desired_time_leds               500               // cada cuanto se actualiza el led

const unsigned long tMuestreoAltura = 100;
const unsigned long tMuestreoSensoresSocket = 100;
const unsigned long tMuestreoEstadosSocket = 100;
const unsigned long tMuestreoI2C = 100;

unsigned long last_time_send_data_client = 0;
unsigned long last_time_send_estado_client = 0;
unsigned long last_time_leds = 0;

unsigned long tInicialMuestreoI2C = 0;
unsigned long tAnteriorMuestreoI2C = 0;
unsigned long tAnteriorMuestreoGPS = 0;
unsigned long tAnteriorMuestreoAltura  = 0;
unsigned long tAnteriorMuestreoSensores = 0;
unsigned long tAnteriorMuestreoEstados = 0;

// ====================================VARIABLES APP===================================================

FtpServer ftpSrv;                           // SERVIDOR FTP CON SD
WiFiServer server(2020);                    // SOCKET
WiFiClient client;

bool CONECTADO_PRIMERA_VEZ = true;
String ARCHIVO_MAPA = "--";    //NOMBRE DEL MAPA A TRABAJAR
int time_out_reset_socket=0;

// ====================================VARIABLES MAPA===================================================
long iteracion_mapa = 0;
long i=0;               //NUMERO DE DATOS ALMACENADOS -1
int ZONA[1000];         //AREGLO PARA GUARDAR LAS ZONAS
int64_t LATITUD[1000];     //AREGLO PARA GUARDAR LATITUD 
int64_t LONGITUD[1000];    //AREGLO PARA GUARDAR LONGITUD
int PWMM[1000];         //AREGLO PARA GUARDAR PWM

// ====================================VARIABLES SD===================================================

File MAPA_FILE;

bool PRIMER_LOG = true;
String LOG_NUM = "--";

// ====================================VARIABLES GPS===================================================

//SFE_UBLOX_GPS GPS_here;

int gps_estado;
int satelites = 0;
int64_t longitud = 0;
int64_t latitud = 0;
uint8_t mes, dia, hora, minuto,segundo;
long anio;
float vel = 0.0;
String latitudString = "", longitudString = "", anioString = "", mesString = "", diaString = "", horaString = "", minutoString = "",segundoString="",estadoGpsString="";
double latitud_fil =  14;
double longitud_fil = -90;

const double CONST_FILTRO_GPS    =  0.8;

// ==========================================VARIABLES TFMINI Y CALCULO DE ALTURA============================================

TFmini tfPro;                         // objeto para tf mini plus
const float CONST_FILTRO_ALTURA = 0.05; // constante del filtro pasabajas, cercano a 1 filtrado es mas rapido atenua menos el ruido, es cercano a 0 atenua mas le ruido pero retrasa la señal
float alturaInst = 0;
String alturaString;

// ========================================================COMUNICACION=================================================================
#define I2C_SDA 21
#define I2C_SCL 22

int porcentaje_flujo_enviar = 100; // 0-100
TwoWire ESP_I2C = TwoWire(0);
byte dato_drone[2]={0,100};
byte flujo_arr[4];
bool enviando_i2c = false;
// ======================================================== FLUJO =================================================================

float flujo = 0;
bool hayFlujoActivo=false;
String flujoString = "0";

// ======================================================== CALIBRACION FLUJO =================================================================

const byte flujo_calibracion_0 = 20 ;// x10 para mmL/s
const byte flujo_calibracion_1 = 70; // x10 para mmL/s
const byte flujo_calibracion_2 = 110 ;// x10 para mmL/s
const byte flujo_calibracion_3 = 160; // x10 para mmL/s
const byte flujo_calibracion_satandby = 250;

bool calibrando = false;

// ======================================================== LED INDICADOR =================================================================

Adafruit_NeoPixel LED_RGB = Adafruit_NeoPixel(1,0, NEO_GRB + NEO_KHZ800);
long ultima_vez_led = 0;
bool led_intermitente = true;

long ultima_vez_tf_leido_correctamente = 3000;
long ultima_vez_GPS_leido_correctamente = 3000;
long ultima_vez_i2c_comunicado_correctamente = 3000;


/////////////////////////////////////////////////// CAN //////////////////////////////////////////////////////////////////////////////////
#define id_estado_nodo                  0x1001550A        // id de la tarjeta en el protocolo dronecan
#define time_to_pub_estado_nodo         1000              // tiempo para enviar heard beat de la esp en dronecan

uint32_t time_estado_nodo = 0;
byte health_byte;
bool guardar_en_buff = false;

int pos_current_array_gps = 0;
byte buff_gps_data[160];
bool nuevo_dato_gps_to_decode = false;
uint64_t fecha_mm;
long anho;
int seg;
double tiempo_seg= 0;
double latitud_anterior= 0;
double longitud_anterior = 0;
double tiempo_calc_vel_gps = 0;

unsigned long last_time_new_dato_gps = 0;
unsigned long last_time_to_pub_estado_nodo =0;


uint32_t CAN_id_frame = 0;
int CAN_tamano_frame = 0;
byte CAN_frame [8];
bool CAN_nuevo_frame_DNA = false;
int fase_DNA = 0;
byte DNA_UNI_ID[19];

void check_estado_de_sistemas();
void setup_led_rgb();
void control_led_HD();
void setup_i2c();
void i2c_intercambio_info_HD();
void setupSd();
void setup_can();
void setupTfMini();
void setupWifi();
String separador(String data, char separator, int index);
int ubica_poligono( int64_t latitude_current, int64_t longitud_current);
bool guardar_ultimo_mapa();
void decode_gps_HD();
void send_estado_nodo_HD();
void get_can(int packetSize);
void sen_can_frame(uint32_t id_nodo, byte buff[], int zize);
void enviar_datos_app_socket();
void wifi_app_HD();
void getTfMiniData();
bool escribirLog(String LOG_NUM, String estadoGps, String latitud, String longitud, float vel, String altura, String flujo);
bool crearArchivoLog();
void leer_dato_app();
void calibrar_serial();
int cargar_mapa_v2 (String DIRECCION_MAPA );
double string_to_long(String numero);
double get_velocidad_gps(double lat1, double long1, double lat2, double long2, double tiempo_seg);
void set_led_can (byte red, byte green, byte blue);

void set_led_can (byte red, byte green, byte blue){
  uint16_t Rgb565 = (((red & 0xf8)<<8) + ((green & 0xfc)<<3)+(blue>>3));
  byte CAN_led_frame[4];
  uint32_t id_led = 0x1204390A;
  CAN_led_frame[0] = 0x00;
  CAN_led_frame[1] = byte(Rgb565 >> 8);
  CAN_led_frame[2] = byte(Rgb565);
  CAN_led_frame[3] = 0xC0;
  sen_can_frame(id_led,CAN_led_frame,4);
}

uint16_t crcAddByte(uint16_t crc_val, uint8_t byte){
    crc_val ^= (uint16_t) ((uint16_t) (byte) << 8U);
    for (uint8_t j = 0; j < 8; j++)
    {
        if (crc_val & 0x8000U)
        {
            crc_val = (uint16_t) ((uint16_t) (crc_val << 1U) ^ 0x1021U);
        }
        else
        {
            crc_val = (uint16_t) (crc_val << 1U);
        }
    }
    return crc_val;
}

uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature){
    for (uint16_t shift_val = 0; shift_val < 64; shift_val = (uint16_t)(shift_val + 8U))
    {
        crc_val = crcAddByte(crc_val, (uint8_t) (data_type_signature >> shift_val));
    }
    return crc_val;
}

uint16_t crcAdd(uint16_t crc_val, const uint8_t* bytes, size_t len){
    while (len--)
    {
        crc_val = crcAddByte(crc_val, *bytes++);
    }
    return crc_val;
}

void DNA_CAN(){
  if(fase_DNA == 0){   
      byte DNA_frame_0[8] = {0x00, CAN_frame[1], CAN_frame[2], CAN_frame[3], CAN_frame[4], CAN_frame[5], CAN_frame[6], 0xC0};
      sen_can_frame(0x1000010A, DNA_frame_0 , 8);
      DNA_UNI_ID [0]= 0xA3;
      DNA_UNI_ID [1]= 0x39;
      DNA_UNI_ID [2]= 0x00;
      for(int i=1; i < 7; i++) {DNA_UNI_ID [2 + i] = CAN_frame[i];}
      fase_DNA = 1;
  }
  else if(fase_DNA == 1){ 
      for(int i=1; i < 7; i++) {DNA_UNI_ID [8 + i] = CAN_frame[i];}
      uint16_t crc = 0xFFFFU;
      crc = crcAdd(crc, DNA_UNI_ID, 15);
      byte DNA_frame_1A[8] = {(uint8_t) crc, (uint8_t) (crc >> 8U), 0x00, DNA_UNI_ID [3], DNA_UNI_ID [4],DNA_UNI_ID [5], DNA_UNI_ID [6], 0x81};
      byte DNA_frame_1B[8] = {DNA_UNI_ID [7], DNA_UNI_ID [8], DNA_UNI_ID [9], DNA_UNI_ID [10], DNA_UNI_ID [11], DNA_UNI_ID [12], DNA_UNI_ID [13], 0x21};   
      byte DNA_frame_1C[2] = {DNA_UNI_ID [14], 0x41};
      sen_can_frame(0x1000010A, DNA_frame_1A, 8);
      sen_can_frame(0x1000010A, DNA_frame_1B, 8);
      sen_can_frame(0x1000010A, DNA_frame_1C, 2);
      fase_DNA = 2;
  }
  else if(fase_DNA == 2){
      
      for(int i=1; i < 6; i++) {DNA_UNI_ID [14 + i] = CAN_frame[i];}
      uint16_t crc = 0xFFFFU;
      DNA_UNI_ID [2] = 0xFA;
      crc = crcAdd(crc, DNA_UNI_ID, 19);
      byte DNA_frame_2A[8] = {(uint8_t) crc, (uint8_t) (crc >> 8U), 0xFA, DNA_UNI_ID [3], DNA_UNI_ID [4],DNA_UNI_ID [5], DNA_UNI_ID [6], 0x82};
      byte DNA_frame_2B[8] = {DNA_UNI_ID [7], DNA_UNI_ID [8], DNA_UNI_ID [9], DNA_UNI_ID [10], DNA_UNI_ID [11], DNA_UNI_ID [12], DNA_UNI_ID [13], 0x22};
      byte DNA_frame_2C[6] = {DNA_UNI_ID [14], DNA_UNI_ID [15], DNA_UNI_ID [16], DNA_UNI_ID [17], DNA_UNI_ID [18], 0x42};
      sen_can_frame(0x1000010A, DNA_frame_2A, 8);
      sen_can_frame(0x1000010A, DNA_frame_2B, 8);
      sen_can_frame(0x1000010A, DNA_frame_2C, 6);
      fase_DNA = 0;
  }
}

void check_estado_de_sistemas(){
  
  if ((millis() - ultima_vez_i2c_comunicado_correctamente) > 2999){
    E_I2C = ERROR_;
  }else{
    E_I2C = OK_;
  }
  if ((millis() - ultima_vez_tf_leido_correctamente) > 2999){
    E_TF = ERROR_;
  }else{
    E_TF = OK_;
  }
  
  if ((millis() - ultima_vez_GPS_leido_correctamente) > 2999){
    E_GPS = ERROR_;
  }
}

void setup_led_rgb(){
  LED_RGB.begin();
  LED_RGB.setPixelColor(0, 0, 250, 250);
  LED_RGB.show();
}

void control_led_HD(){ 
  if((millis()-ultima_vez_led) > 500){ //ejecutar cada 0.5 seg
    led_intermitente = !led_intermitente;
    if (calibrando){
      if(led_intermitente){
          LED_RGB.setPixelColor(0, 127, 0, 255);
          set_led_can (0,127,255);
        }else{
          LED_RGB.setPixelColor(0, 255, 0, 127);
          set_led_can (255,0,127);}
          
    }else{
      if(E_I2C == ERROR_){
        //secuencia_a_mostrar = ROJO_ROJO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);
        }else{
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);}
          
      }else if(E_BOMBAS == ERROR_){ // parpadeo ROJO AMARILLO
        //secuencia_a_mostrar = ROJO_AMARILLO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);
        }else{
          LED_RGB.setPixelColor(0, 255, 255, 0);
          set_led_can (255,255,0);}
      }else if(E_GPS == ERROR_){
        //secuencia_a_mostrar = ROJO_BLANCO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);
        }else{
          LED_RGB.setPixelColor(0, 255, 255, 255);
          set_led_can (255,255,255);}
      }else if(E_SD == ERROR_){
        //secuencia_a_mostrar = ROJO_CIAN;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);
        }else{
          LED_RGB.setPixelColor(0, 0, 255, 255);
          set_led_can (0,255,255);}
      }else if(E_TF == ERROR_){
        //secuencia_a_mostrar = ROJO_MORADO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 255, 0, 0);
          set_led_can (255,0,0);
        }else{
          LED_RGB.setPixelColor(0, 255, 0, 255);
          set_led_can (0,127,255);}
      }else if(E_MAPA == NO_CARGADO){
        //secuencia_a_mostrar = CIAN_APAGADO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 0, 255, 255);
          set_led_can (0,255,255);
        }else{
          LED_RGB.setPixelColor(0, 0, 0, 0);
          set_led_can (0,0,0);}
      }else if(E_GPS == BUSCANDO){
        //secuencia_a_mostrar = VERDE_APAGADO;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 0, 255, 0);
          set_led_can (0,255,0);
        }else{
          LED_RGB.setPixelColor(0, 0, 0, 0);
          set_led_can (0,0,0);}
      }else if(E_LOG == LISTO_PARA_GUARDAR && E_GPS == OK_){
        //secuencia_a_mostrar = VERDE_VERDE;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 0, 255, 0);
          set_led_can (0,255,0);
        }else{
          LED_RGB.setPixelColor(0, 0, 255, 0);
          set_led_can (0,255,0);}
      }else if(E_LOG == GUARDANDO && E_GPS == OK_){
        //secuencia_a_mostrar = VERDE_AZUL;
        if(led_intermitente){
          LED_RGB.setPixelColor(0, 0, 255, 0);
          set_led_can (0,255,0);
        }else{
          LED_RGB.setPixelColor(0, 0, 0, 255);
          set_led_can (0,0,255);}
      }
    }
    LED_RGB.show();
    ultima_vez_led = millis();
  }
}

void setup_i2c(){
  ESP_I2C.begin(I2C_SDA, I2C_SCL);
  ESP_I2C.setTimeOut(1);
}

void set_porcentaje_i2c(){
    dato_drone[0] = E_DRONE;
    dato_drone[1] = porcentaje_flujo_enviar;  
    ESP_I2C.beginTransmission(8);
    ESP_I2C.write(dato_drone,sizeof(dato_drone));
    ESP_I2C.endTransmission();
    //Serial.println("flujo deseado: " + String(porcentaje_flujo_enviar) +" estado dron: " + String(E_DRONE));    
  
    ESP_I2C.requestFrom(8,5);
    if (ESP_I2C.available()) {
      for (int i=0;i<sizeof(flujo_arr);i++){
        flujo_arr[i]= ESP_I2C.read();
      }
      E_BOMBAS = ESP_I2C.read();
      float flujo_temp = *((float*)flujo_arr);
      
      if (!isnan(flujo_temp)) {
        flujo = flujo_temp;
        flujoString = String(flujo/1000);
        if (flujo>0.01){
          hayFlujoActivo = true;
        }else{
          hayFlujoActivo = false;
        }
        ultima_vez_i2c_comunicado_correctamente = millis();
        //Serial.println("flujo BOMBAS: " + flujoString +" estado BOMBAS: " + String(E_BOMBAS));
      }
    }
  
}

void i2c_intercambio_info_HD(){
  tInicialMuestreoI2C = millis();
  if(((tInicialMuestreoI2C-tAnteriorMuestreoI2C) > tMuestreoI2C) && !enviando_i2c){
    set_porcentaje_i2c();
    tAnteriorMuestreoI2C =  millis();
  }
}

void setupSd(){
  if (!SD.begin(2)){
    E_SD = ERROR_;
    Serial.print("error SD");
    for(;;){
      LED_RGB.setPixelColor(0, 255, 0, 0);
      LED_RGB.show();
      delay(500);
      LED_RGB.setPixelColor(0, 255, 0, 255);
      LED_RGB.show();
      delay(500);
    }
  }
  else
    E_SD = LISTO_PARA_GUARDAR;  
}

void setup_can(){
  CAN.setPins(4,5);
  if (!CAN.begin(1000E3)) {
    while (1);
  }
  CAN.onReceive(get_can);
}

void setupTfMini(){
  Serial1.begin(TFmini::DEFAULT_BAUDRATE, SERIAL_8N1, 13, 14);
  tfPro.attach(Serial1);
}

void setupWifi(){
  WiFi.mode(WIFI_AP_STA); 
  //WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid,password);
  Serial.println(WiFi.softAPIP());
  Serial.println(WiFi.macAddress());
  server.begin();
  ftpSrv.begin(username_server,password_server);
}

    void loop2(void *pvParameters){
      for (;;){
        if (!calibrando){i2c_intercambio_info_HD();}
        else{hayFlujoActivo = false;}   
        control_led_HD();
        wifi_app_HD();
        getTfMiniData();  
      }
    }

int cargar_mapa_v2 (String DIRECCION_MAPA ){
  File MAPA_FILE_TRABAJO = SD.open(DIRECCION_MAPA);
  String filas_leida;
  iteracion_mapa = 0;
  
  if (MAPA_FILE_TRABAJO) {
    while (MAPA_FILE_TRABAJO.available()){
       filas_leida = MAPA_FILE_TRABAJO.readStringUntil('\n');
       String ZONA_String = separador(filas_leida,',',0);
       ZONA [iteracion_mapa] = string_to_long(ZONA_String);
       Serial.print(String(ZONA [iteracion_mapa]) + ",");
       
       String LATITUD_String = separador(filas_leida,',',1);
       LATITUD [iteracion_mapa] = int64_t(LATITUD_String.toDouble()*10000000);
       Serial.print(String(LATITUD [iteracion_mapa]) + ",");
       
       String LONGITUD_String = separador(filas_leida,',',2);
       LONGITUD [iteracion_mapa] = int64_t(LONGITUD_String.toDouble()*10000000);
       Serial.print(String(LONGITUD [iteracion_mapa]) + ",");
       
       String PWM_String = separador(filas_leida,',',3);
       PWMM [iteracion_mapa] = PWM_String.toInt();
       Serial.println(String(PWMM [iteracion_mapa]));
       
       iteracion_mapa ++;
       if (iteracion_mapa > (sizeof(ZONA)/sizeof(ZONA[0]))){
        Serial.println("error demasiados datos");
        MAPA_FILE_TRABAJO.close();
        return 2;
       }
    }
    MAPA_FILE_TRABAJO.close();
    Serial.println("iteracion_mapa: " + String(iteracion_mapa));
    return 1;
  }else{
    Serial.println("error no se pudo abrir mapa");
    return 3;
  }
}

double string_to_long(String numero){
  int i_dat = 0;
  long dato_long_temporal;
  String dato_string_temporal;
  do{
    dato_string_temporal = numero.substring(i_dat);
    dato_long_temporal = dato_string_temporal.toInt();
    i_dat++;
    if(i_dat > 50){
      return 0;
    }
  }while(dato_long_temporal == 0);
  
  return dato_long_temporal;
}

int ubica_poligono( int64_t latitude_current, int64_t longitud_current){ //REGRESA EL PORCENTAJE DE AVERTURA DE LA VALVULA QUE LE CORRESPONDE, INGRESA POSICION ACTUAL DEL DRONE
  int zoneanterior = PWMM[0];        // zona anterior revisada
  int lim_inf=0;                  // limete inferior de una zona N
  int lim_sup=0;                    // limite superior de una zona N
  int CURRENT_PWM=0;              // flujo actual deacuerdo a la zona, resultado de la funcion
  bool punto_fuera=true;          // si la posiscion ingresada esta dentro del poligo es verdadero

  for (int I=0;I<iteracion_mapa;I++){
    if (PWMM[I]!=zoneanterior || (iteracion_mapa-1)==I){
        if ((iteracion_mapa-1)==I){
          lim_sup = I;
        }else{
          lim_sup = I-1;
        }
        //Serial.println("NUEVA ZONA: zona = " + String(ZONA[lim_inf]) + " limite inferior = " + String(lim_inf) + " lim_sup = " + " " +String(lim_sup));
        bool salida=false;
        int inf_i,inf_j=0;
        
        for (inf_i = lim_inf,inf_j = (lim_inf + (lim_sup - lim_inf)); inf_i <= lim_sup; inf_j = inf_i++) {
          //Serial.println("inf_i = " + String(inf_i) + " inf_j = " + String(inf_j));
          if (((LONGITUD[inf_i]>longitud_current) != (LONGITUD[inf_j]>longitud_current))){
            //Serial.println("entre primer if B");
            if ((latitude_current < (LATITUD[inf_j]-LATITUD[inf_i]) * (longitud_current-LONGITUD[inf_i]) / (LONGITUD[inf_j]-LONGITUD[inf_i]) + LATITUD[inf_i]) ){
              salida = !salida;  
              //Serial.println("entre segundo if B");            
            }
          }
        }
        if (salida){
          CURRENT_PWM=PWMM[lim_inf];
          //Serial.println("PUNTO DENTRO DEL POLIGONO: " + String(ZONA[lim_inf]));                      
          punto_fuera=false;
          return CURRENT_PWM;                             // RETORNA EL PORCENTAJE QUE LE CORRESPONDE CUANDO SE ENCUENTRA DENTRO DE UN POLIGONO
          break;
        }
        lim_inf=lim_sup+1;
      }
      zoneanterior=PWMM[I];
    }
    if (punto_fuera){
      //Serial.println("PUNTO PFUERA");
      CURRENT_PWM = 100;
      return CURRENT_PWM;                                  // SI NO ESTA DENTRO DE UN POLIGONO RETORNA EL 100%, PARA PERMITIR EL FLUJO MAXIMO
    }
}

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

void guardar_ultimo_mapa(String ultimo_mapa){
  File ULTIMO_MAPA_FILE = SD.open("/ULTIMO_M_USADO.txt", FILE_WRITE);
  if (ULTIMO_MAPA_FILE){
    ULTIMO_MAPA_FILE.println(ultimo_mapa);
    ULTIMO_MAPA_FILE.close();
    //Serial.println("ultimo mapa guardado: " + ultimo_mapa);
  }else{
    Serial.println("Error al guardar ultimo mapa.");
  }  
}

void decode_gps_can(){
    byte fecha[8]; //8
    fecha[0] = buff_gps_data[9];
    fecha[1] = buff_gps_data[10];
    fecha[2] = buff_gps_data[11];
    fecha[3] = buff_gps_data[12];
    fecha[4] = buff_gps_data[13];
    fecha[5] = buff_gps_data[14];
    fecha[6] = buff_gps_data[15];
    fecha[7] = 0x00;
    fecha_mm = ((*((uint64_t*) fecha))/1000000)+ 2740143;
    anho = 1970 + floor((fecha_mm)/ 31556929);
    long mes_s = (fecha_mm)% 31556929;
    mes = floor(mes_s/2629743); // 2629743
    long dia_s = mes_s % 2629743;
    dia = floor(dia_s/86400); // 2x 86400 172800
    long hora_s = dia_s % 86400;
    hora = floor(hora_s/3600);
    long minu_s = hora_s % 3600;
    minuto =minu_s/60; //13*60 780
    seg =minu_s%60; //13*60 780
    
   
    satelites = buff_gps_data[47]>>2;
    
    gps_estado = buff_gps_data[47] & B00000011;
    
    byte longitud_arr[8];
    longitud_arr[0] = buff_gps_data[19];
    longitud_arr[1] = buff_gps_data[20];
    longitud_arr[2] = buff_gps_data[21];
    longitud_arr[3] = buff_gps_data[22];
    longitud_arr[4] = (buff_gps_data[23] & B01111111)>>3;
    longitud_arr[5] = 0x00;
    longitud_arr[6] = 0x00;
    longitud_arr[7] = 0x00;
    longitud = *((int64_t*) longitud_arr);
  
    if ((buff_gps_data[23] >> 7) == 0x01){
      longitud = longitud - 68719476736;
    }
    longitud = longitud/10;
  
    byte latitud_arr[8];
    latitud_arr[0] = (buff_gps_data[23]<< 5) | (buff_gps_data[24]>>3);
    latitud_arr[1] = (buff_gps_data[24]<< 5) | (buff_gps_data[25]>>3);
    latitud_arr[2] = (buff_gps_data[25]<< 5) | (buff_gps_data[26]>>3);
    latitud_arr[3] = (buff_gps_data[26]<< 5) | (buff_gps_data[27]>>3);
    latitud_arr[4] = (buff_gps_data[27]<< 5) | ((buff_gps_data[28] & 0xC0)>>3);
    latitud_arr[4] = (latitud_arr[4]>>3) & 0x0F;
    latitud_arr[5] = 0x00;
    latitud_arr[6] = 0x00;
    latitud_arr[7] = 0x00;
    latitud = *((int64_t*) latitud_arr);

    if ((latitud_arr[4]>>4) == 0x01){
      latitud = latitud - 68719476736;
    }
    latitud = latitud/10;
  
  
  double latitudAux = double(latitud) / 10000000.0;
  latitud_fil = CONST_FILTRO_GPS * latitudAux + (1 - CONST_FILTRO_GPS) * latitud_fil;
  latitudString = String(latitud_fil, 7);
      
  double longitudAux =  double(longitud) / 10000000.0;
  longitud_fil = CONST_FILTRO_GPS * longitudAux + (1 - CONST_FILTRO_GPS) * longitud_fil;
  longitudString = String(longitud_fil, 7);

  tiempo_seg = double(micros() - tiempo_calc_vel_gps);
  tiempo_seg = tiempo_seg/1000000;
  
  vel = get_velocidad_gps(latitud_anterior,longitud_anterior,latitudAux,longitudAux,tiempo_seg) * 3.6; //en km/hora
  
  latitud_anterior = latitudAux;
  longitud_anterior = longitudAux;
  tiempo_calc_vel_gps = micros();
    
  if(satelites >= 8 ){E_GPS = OK_;}
  else{E_GPS = BUSCANDO;}
  
  estadoGpsString=String(gps_estado);

  if(satelites <= 0){
    E_GPS = ERROR_;
  }else{
    ultima_vez_GPS_leido_correctamente = millis();
  }
  
  diaString = String(dia);
  mesString = String(mes);
  anioString = String(anho);
  horaString = String(hora);
  if(minuto < 10){minutoString = "0" + String(minuto);}
  else{minutoString = String(minuto);}
  if(seg < 10){segundoString = "0" + String(seg);}
  else{segundoString = String(seg);}
  nuevo_dato_gps_to_decode = false;
  //Serial.println("Estado: "+estadoGpsString+ " Latitud fill: "+ latitudString +" Longitud fill: "+ longitudString + " Latitud: "+ String(latitud) +" Longitud: "+ String(longitud) +" satelites "+ String(satelites)+" velocidad "+ String(vel)+" seg "+ String(seg));
  //Serial.println(latitudString + ","+ String(latitudAux,7));
}

void CAN_HD(){
  if(nuevo_dato_gps_to_decode){
    decode_gps_can();  
    if(E_DRONE==AIRE ){
      enviando_i2c = true;
      if(E_MAPA != NO_CARGADO){
        porcentaje_flujo_enviar = ubica_poligono(latitud,longitud);
        Serial.println("mapa flujo: " + String(porcentaje_flujo_enviar));
      }
      set_porcentaje_i2c();
      if (escribirLog(LOG_NUM, estadoGpsString, latitudString, longitudString, vel, alturaString, flujoString)){
        E_LOG = GUARDANDO; 
        E_SD = OK_;
      }else{
        E_SD = ERROR_;
        E_LOG = ERROR_;
      }
      enviando_i2c = false;
    }else{
      E_LOG = LISTO_PARA_GUARDAR;
    }    
  }
  if(CAN_nuevo_frame_DNA){
    DNA_CAN();
    CAN_nuevo_frame_DNA = false;
  }
}

void get_can(int packetSize){
    CAN_id_frame = CAN.packetId();
    CAN_tamano_frame = packetSize;
    
    if(CAN_id_frame == 0X804277D){                                          // guarda datos para gps 
      while(CAN.available()){
        buff_gps_data[pos_current_array_gps]= CAN.read();
        pos_current_array_gps++;
      }
      
      if(buff_gps_data[pos_current_array_gps-1]>>6 == 0x01){
        if (pos_current_array_gps < 79){nuevo_dato_gps_to_decode = true;}
        pos_current_array_gps=0;
      }
      else{pos_current_array_gps--;}
    } 
    else if ((CAN_id_frame & 0xFF0000FF) == 0x18000000){                                                                   // guardar datos para enviar respuesta de la esp
      for(int i=0; i<packetSize ; i++){CAN_frame[i] = CAN.read();}
      CAN_nuevo_frame_DNA = true;
    }
}

void send_estado_nodo_HD(){
  if ((millis()-last_time_to_pub_estado_nodo) > time_to_pub_estado_nodo){
    if(time_estado_nodo == 0){health_byte=B00001000;}
    else{health_byte=B00000000;}
    
    byte buff_estado_nodo[8];
    byte * time_estado_nodo_arr_byte = (byte *) &time_estado_nodo;
    
    buff_estado_nodo[0]= time_estado_nodo_arr_byte[0];
    buff_estado_nodo[1]= time_estado_nodo_arr_byte[1];
    buff_estado_nodo[2]= time_estado_nodo_arr_byte[2];
    buff_estado_nodo[3]= time_estado_nodo_arr_byte[3];
    buff_estado_nodo[4]= health_byte;
    buff_estado_nodo[5]= 0x00; 
    buff_estado_nodo[6]= 0x00;
    buff_estado_nodo[7]= B11000000 | (time_estado_nodo_arr_byte[0] & B00011111);
    
    sen_can_frame(id_estado_nodo,buff_estado_nodo,8);
    time_estado_nodo ++;
    last_time_to_pub_estado_nodo = millis();
  }
}

void sen_can_frame(uint32_t id_nodo, byte buff[], int zize){
  CAN.beginExtendedPacket(id_nodo);
    for(int i=0; i<zize; i++){
      CAN.write(buff[i]);
    }
    CAN.endPacket();
}

double get_velocidad_gps(double lat1, double long1, double lat2, double long2, double tiempo_seg){
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  double distancia = delta * 6372795; 
  return distancia/tiempo_seg;
}

void setup(){
  //delay(5000);
  Serial.begin(500000);
  Serial.println("Serial ok");
  setup_led_rgb();
  Serial.println("LED OK");
  setupSd();
  Serial.println("SD Ok");  
  setupTfMini();
  Serial.println("TFmini Ok");
  xTaskCreatePinnedToCore(loop2, "Task1", 5000, NULL, 0, &Task1, 0);
  Serial.println("Segundo core Ok");
  setup_i2c();
  Serial.println("Comunicacion i2c Ok"); 
  setupWifi();
  Serial.println("Comunicacion WIFI Ok"); 
  setup_can();
  
  if(SD.exists("/ULTIMO_M_USADO.txt")){
    File ULTIMO_MAPA_FILE = SD.open("/ULTIMO_M_USADO.txt");
    if (ULTIMO_MAPA_FILE){
      if (ULTIMO_MAPA_FILE.available()){
        String ARCHIVO_MAPA_ = ULTIMO_MAPA_FILE.readStringUntil('.');
        ULTIMO_MAPA_FILE.close();
        
        ARCHIVO_MAPA_ = ARCHIVO_MAPA_ + ".csv";
        Serial.println("ULTIMAPA USADO FUE:  " + ARCHIVO_MAPA_);
        if(SD.exists(ARCHIVO_MAPA_)){
          if (cargar_mapa_v2(ARCHIVO_MAPA_) != 1){
            Serial.println("Error al cargar el mapa_.");
            E_MAPA = NO_CARGADO;          
          }else{
            Serial.println("Se ha cargado el mapa exitosamente._");
            E_MAPA = MAPA_PREVIO_CARGADO;
            ARCHIVO_MAPA = ARCHIVO_MAPA_;
          }
        }else{
          Serial.println("Error al cargar el mapa.");
          E_MAPA = NO_CARGADO;
        }
      }else{
        ULTIMO_MAPA_FILE.close();
        Serial.println("Error al cargar el mapa_.");
        E_MAPA = NO_CARGADO; 
      }
    }else{
      ULTIMO_MAPA_FILE.close();
      Serial.println("Error al cargar el mapa_.");
      E_MAPA = NO_CARGADO; 
    }
    
  }else{
    Serial.println("Error al cargar el mapa_.");
    E_MAPA = NO_CARGADO; 
  }

  if(SD.exists("/ULTIMO_L_O_G.txt")){
    File NUM_LOG_FILE = SD.open("/ULTIMO_L_O_G.txt");
    if (NUM_LOG_FILE){
      if (NUM_LOG_FILE.available()){
        LOG_NUM = NUM_LOG_FILE.readStringUntil('.');
        LOG_NUM = LOG_NUM + ".txt";
        Serial.println("guardando en ultimo log:  " + LOG_NUM);
        E_LOG = LISTO_PARA_GUARDAR;
      }else{
        Serial.println("ARCHIVO NUEVO LOG VACIO");
      }
      NUM_LOG_FILE.close();
    }else{
      if (crearArchivoLog()){
          E_LOG = LISTO_PARA_GUARDAR;
          Serial.println("Nuevo log creado " + LOG_NUM);
      }else{
        E_SD = ERROR_;
        E_LOG = ERROR_;
        Serial.println("No Se pudo iniciar log");
      }
    }
    
    
  }else{
    if (crearArchivoLog()){
        E_LOG = LISTO_PARA_GUARDAR;
        Serial.println("Nuevo log creado" + LOG_NUM);
    }else{
      E_SD = ERROR_;
      E_LOG = ERROR_;
      Serial.println("No Se pudo iniciar log");
    }
  }


}

void enviar_datos_app_socket(){
  if((millis()-tAnteriorMuestreoSensores) > tMuestreoSensoresSocket){
    client.print("1," + String(porcentaje_flujo_enviar) + "," + satelites);
    client.print("," + latitudString + "," + longitudString);
    client.print("," + String(vel) + "," + String(alturaInst) + "," + flujoString);
    client.println("," + anioString + "/" + mesString + "/" + diaString + "," + horaString + ":" + minutoString);
    tAnteriorMuestreoSensores =  millis();
  }      
  if((millis()-tAnteriorMuestreoEstados) > tMuestreoEstadosSocket){
    client.print("2," + ARCHIVO_MAPA + "," + LOG_NUM);
    client.println("," + String(E_SD) + "," + estadoGpsString + "," + String(E_DRONE) + "," + String(E_MAPA) + "," + String(E_LOG));
    tAnteriorMuestreoEstados =  millis();
  }
}

void wifi_app_HD(){
  ftpSrv.handleFTP();
  if (CONECTADO_PRIMERA_VEZ){
    client = server.available();
    if (client){
      CONECTADO_PRIMERA_VEZ = false;
    }
  }
  if (client && !CONECTADO_PRIMERA_VEZ){
    if (client.connected()){
      leer_dato_app();
      if (!calibrando){
        enviar_datos_app_socket();
      }
    }
    E_SOCKET = SOCKET_CONECTADO;
  }else {
    if (time_out_reset_socket > 200){
      client.stop();
      E_SOCKET = SOCKET_LISTO;
      CONECTADO_PRIMERA_VEZ = true;
      time_out_reset_socket=0;
    }
    time_out_reset_socket++;
  }
}

void getTfMiniData(){
  if ((millis() - tAnteriorMuestreoAltura) > tMuestreoAltura){    
    if (tfPro.available()){
      ultima_vez_tf_leido_correctamente = millis();
      alturaInst = CONST_FILTRO_ALTURA * float(tfPro.getDistance()) + (1 - CONST_FILTRO_ALTURA) * alturaInst;
      
      if(alturaInst < 50){
        if (E_DRONE==AIRE){
          if(vel < 0.4){
            E_DRONE=TIERRA; 
          }
        }else{
          E_DRONE=TIERRA;
        }
      }
      else
        E_DRONE=AIRE;

      alturaString= String(alturaInst/100.0,1);

      //Serial.println("Altura Instantanea: " + String(alturaInst) + "  AlturaProm: " + alturaString);

    }
    tAnteriorMuestreoAltura = millis();
  }
}

bool crearArchivoLog(){
  long num = 0;
  do{
    num++;
    LOG_NUM = "/addlog-" + diaString + "-" + mesString + "-" + anioString + "_" + String(num) + ".txt";
  } while (SD.exists(LOG_NUM));

  /////////////////////////// guardando el nombre del ultimo log ///////////////////////////////
  File NUM_LOG_FILE = SD.open("/ULTIMO_L_O_G.txt", FILE_WRITE);
  if (NUM_LOG_FILE){
    NUM_LOG_FILE.println(LOG_NUM);
    NUM_LOG_FILE.close();
  }
  else{
    return false;
  }

  ////////////////////////////////// crea cabecera del log /////////////////////////////////////
  MAPA_FILE = SD.open(LOG_NUM, FILE_WRITE);
  if (MAPA_FILE){
    MAPA_FILE.println("SATELITES,LATITUD,LONGITUD,VELOCIDAD(Km/h),ALTURA(m),FLUJO(L/min),FECHA,HORA");
    MAPA_FILE.close();
    return true;
  }
  else{
    MAPA_FILE.close();
    return false;
  }
}

bool escribirLog(String LOG_NUM, String estadoGps, String latitud, String longitud, float vel, String altura, String flujo){
  MAPA_FILE = SD.open(LOG_NUM, FILE_APPEND);
  if (MAPA_FILE){
    MAPA_FILE.println(String(satelites) + ", " + latitud + ", " + longitud + ", " + String(vel) + ", " + altura + ", " + flujo+", " + diaString + "/" + mesString + "/" + anioString + ", " + horaString + ":" + minutoString + ":" + segundoString);
    MAPA_FILE.close();
    return true;
  }
  else{
    MAPA_FILE.close();
    return false;
  }
}

void leer_dato_app(){   
  if(client.available()){ //client
    String dato_client = client.readStringUntil('\n'); //client
    String modo_cliente = separador(dato_client,',',0);
    int modo_cliente_int = modo_cliente.toInt();
    String comando_cliente = separador(dato_client,',',1);
    
    Serial.println("dato recibido desde la app  MODO: " + String(modo_cliente_int) + comando_cliente);
    if(modo_cliente_int == 4){
      int comando_char = comando_cliente.toInt();
      switch (comando_char) {
      case 5:    // your hand is on the sensor
        Serial.println("GUARDAR CALIBRACION_");
        dato_drone[0] = GUARDAR_CALIBRACION;
        dato_drone[1] = flujo_calibracion_satandby;
        calibrando = false; 
        break;
      case 0:    // your hand is on the sensor
        Serial.println("INICIO DE CALIBRACION_");
        dato_drone[0] = CALIBRACION;
        dato_drone[1] = flujo_calibracion_0;
        calibrando = true;
        break;
      case 1:    // your hand is on the sensor
        Serial.println("calibrar punto 1_");
        dato_drone[0] = CALIBRACION;
        dato_drone[1] = flujo_calibracion_0; 
        break;
      case 2:    // your hand is close to the sensor
        Serial.println("calibrar punto 2_");
        dato_drone[0] = CALIBRACION;
        dato_drone[1] = flujo_calibracion_1;
        break;
      case 3:    // your hand is a few inches from the sensor
        Serial.println("calibrar punto 3_");
        dato_drone[0] = CALIBRACION;
        dato_drone[1] = flujo_calibracion_2;
        break;
      case 4:    // your hand is nowhere near the sensor
        Serial.println("calibrar punto 4_");
        dato_drone[0] = CALIBRACION;
        dato_drone[1] = flujo_calibracion_3;
        break;
      }
    
      ESP_I2C.beginTransmission(8);
      ESP_I2C.write(dato_drone,sizeof(dato_drone));
      ESP_I2C.endTransmission();
      Serial.println("estado calibrando: " + String(dato_drone[0]) +" flujo a calibrar: " + String(dato_drone[1] + "_"));    

      byte respuesta_calibracion_array[4];
      ESP_I2C.requestFrom(8, sizeof(respuesta_calibracion_array));
    
      if (ESP_I2C.available()) {
        for (int i=0;i<sizeof(respuesta_calibracion_array);i++){respuesta_calibracion_array[i]= ESP_I2C.read();}
        float respuesta_calibracion = *((float*)respuesta_calibracion_array);
        
        if (!isnan(respuesta_calibracion)) {
          if(comando_char == 0){
            client.println("4,0");
            Serial.println("4,5");
          }else if(comando_char == 5){
            client.println("4,5");
            Serial.println("4,5");
          }else{
            client.println("4," + comando_cliente + "," + String(respuesta_calibracion));
            Serial.println("4," + comando_cliente + "," + String(respuesta_calibracion));
          }     
        }
      }
    }
    else if(modo_cliente_int == 5){
      Serial.println("Nombre de mapa recibido, intentando cargar el mapa " + comando_cliente +"..._");
      String comando_cliente_ = "/" + comando_cliente;
      
      if (cargar_mapa_v2(comando_cliente_) != 1)
        Serial.println("Error al cargar el mapa_.");          
      else{
        ARCHIVO_MAPA = comando_cliente_;
        client.println("3," + comando_cliente);//client
        Serial.println("Se ha cargado el mapa exitosamente._");
        
        guardar_ultimo_mapa(comando_cliente_);
        E_MAPA=MAPA_CARGADO_DESDE_APP;
      }
    }
    else if(modo_cliente_int == 6){
      if (crearArchivoLog()){
        E_LOG = LISTO_PARA_GUARDAR;
        String LOG_NUM_APP = LOG_NUM.substring(1);
        Serial.println("Nuevo log creado_" + LOG_NUM_APP);
        client.println("6," + LOG_NUM_APP); //client
      }else{
        E_SD = ERROR_;  
        Serial.println("ERROR, NO SE PUDO CREAR LOG_");
        client.println("6,0"); //client
      }
    }
  
  } 
}

void calibrar_serial(){
  if(Serial.available()){
    int S = Serial.read();
    Serial.read();
    Serial.print(S);
    switch (S) {
    case 'g':    // your hand is on the sensor
      Serial.println("GUARDAR CALIBRACION");
      dato_drone[0] = GUARDAR_CALIBRACION;
      dato_drone[1] = flujo_calibracion_satandby;
      calibrando = false; 
      break;
    case 'c':    // your hand is on the sensor
      Serial.println("INICIO DE CALIBRACION");
      dato_drone[0] = CALIBRACION;
      dato_drone[1] = flujo_calibracion_satandby;
      calibrando = true;
      break;
    case '1':    // your hand is on the sensor
      Serial.println("calibrar punto 1");
      dato_drone[0] = CALIBRACION;
      dato_drone[1] = flujo_calibracion_0; 
      break;
    case '2':    // your hand is close to the sensor
      Serial.println("calibrar punto 2");
      dato_drone[0] = CALIBRACION;
      dato_drone[1] = flujo_calibracion_1;
      break;
    case '3':    // your hand is a few inches from the sensor
      Serial.println("calibrar punto 3");
      dato_drone[0] = CALIBRACION;
      dato_drone[1] = flujo_calibracion_2;
      break;
    case '4':    // your hand is nowhere near the sensor
      Serial.println("calibrar punto 4");
      dato_drone[0] = CALIBRACION;
      dato_drone[1] = flujo_calibracion_3;
      break;
    }
    
    ESP_I2C.beginTransmission(8);
    ESP_I2C.write(dato_drone,sizeof(dato_drone));
    ESP_I2C.endTransmission();
    Serial.println("estado calibrando: " + String(dato_drone[0]) +" flujo a calibrar: " + String(dato_drone[1]));    
  
    ESP_I2C.requestFrom(8, sizeof(flujo_arr));
    if (ESP_I2C.available()) {
      for (int i=0;i<sizeof(flujo_arr);i++){
        flujo_arr[i]= ESP_I2C.read();
      }
      float flujo_temp = *((float*)flujo_arr);
      
      if (!isnan(flujo_temp)) {
        flujo = flujo_temp;
        Serial.println("respuesta esp " + String(flujo));
      }
    }
  }
}

void loop(){
  CAN_HD();
  check_estado_de_sistemas();
  send_estado_nodo_HD();
}
