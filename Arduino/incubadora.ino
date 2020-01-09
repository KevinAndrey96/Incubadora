
 #include <EEPROM.h>
#include <Wire.h>
#include "Nextion.h"
#include <HX711.h>
#include <dht.h>
dht DHT;
#include <RTClib.h>
RTC_DS3231 rtc;

#include <OneWire.h>
#include <DallasTemperature.h>

#include <SoftwareSerial.h>
SoftwareSerial SIM900(10, 11);
SoftwareSerial mostar(15, 14);//rx,tx del sensor de c02
#define pin_temperatura     3
#define carga              13
#define led                 12
///actuadores y pines de sensores
  #define ven1 22 //ventilador horizontal 1
  #define ven2 23 //ventilador horizontal 2
  #define ven3 24 //ventilador horizontal 3
  #define ven4 25 //ventilador extractor
  #define ven5 26 //ventilador del damper
  #define ven6 49 //ventilador del disipador
  #define damper 27 //Rejilla de damper
  #define r1 28 //Resistencia de calefaccion 1
  #define r2 29 //Resistencia de calefaccion 2
  #define mb 30 //Motobomba
  #define hum 31 //Humidificador
  #define ca 32 //Calentador de agua
  #define sn 33 //Sensor de |||||||||||||||||||||||||||||<t5rrrrrrr6baaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaHqqqqqqqqqqqqqqtgyyyyyyyysen el humidificador
  #define fcp 34 // final de carrera de la puerta
  #define sh 46 //pin de conmutacion de sensores de humedad
  #define lampled 47 //lampara led
  #define lampuv 48 //lampara UV
  #define mq A0 //sensonr de calidad de aire
  #define led1 39 //giro del motor en un sentido
  #define led2 40 //giro del motor en sentido contrario
  #define led3 41 //Indicador de funcionamiento
  #define in 42  //Bot+on de arranque
//Fin definicion de actuadores 
//comuniccion temperatura
///puntos

float Setpoint=29.0,hd=60.0,Setpoint1,Setpoint2,hd1,hd2;
float rot1,rot2,rot3;
OneWire comuniacion(pin_temperatura);
DallasTemperature ds18b20(&comuniacion);

//--------------------------------------------Variables de temperatura
float t3,t2,tp;
float temperatura = 0;

OneWire ourWire1(50);                //Se establece el pin 2  como bus OneWire
OneWire ourWire2(52);                //Se establece el pin 3  como bus OneWire
DallasTemperature sensors1(&ourWire1); //Se declara una variable u objeto para nuestro sensor1
DallasTemperature sensors2(&ourWire2); //Se declara una variable u objeto para nuestro sensor2
//--------------------------------------------Fin variables temperatura
//-------------------------------------------humedad
String calaire;
#define DHT22_PIN 7
#define DHT22_PIN1 8
int adc=0;
float hu,hu1,hup;
struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0};
//--------------------------------------------
//------------------------------------presion
const int LOADCELL_DOUT_PIN = 42;
const int LOADCELL_SCK_PIN = 43;
const int LOADCELL_DOUT_PIN1 = 44;
const int LOADCELL_SCK_PIN2 = 45;
HX711 scale;
HX711 scale1;
float p1,p2,pp;
//------------------------------------
//----------------------------milis 
unsigned long t = 0;
const long intervalo = 1200;
 bool bandera=false;
 int mot1=0,mot2=0;
  
//-------------------------
//utra sonido------------------------------------
int TRIG = 4;      // trigger en pin 10
int ECO = 5;      // echo en pin 9
int LED = 13;      // LED en pin 3
int DURACION;
int DISTANCIA;
int altura;
int niveldeagua;
int aux1=18;
float niveldeagua1;
//-----------------------------------------------------
///////RTC RELOJ


int flag=0,d,h,m=-1,s,aux=110,sg=0,as;
String q="";//ENVIO CADENA DE DATOS;


///// VARIABLES Y CONFIGURACION PANTALLA NEXTION 
NexNumber tep    = NexNumber(1, 6, "n0");
NexNumber te1    = NexNumber(1, 7, "n1");
NexNumber te2    = NexNumber(1, 8, "n2");
NexNumber hp    = NexNumber(1, 9, "n3");
NexNumber h1    = NexNumber(1, 10, "n4");
NexNumber h2    = NexNumber(1, 11, "n5");
NexNumber nivel    = NexNumber(1, 12, "n6");
NexNumber pe1    = NexNumber(1, 13, "n7");
NexNumber pe2    = NexNumber(1, 14, "n8");
NexNumber pep    = NexNumber(1, 16, "n10");
NexNumber naci    = NexNumber(1, 15, "n9");
NexProgressBar temp  = NexProgressBar(1, 1, "j0");
NexProgressBar niel  = NexProgressBar(1, 2, "j1");
///botones-----------------------------
NexDSButton com = NexDSButton(0, 16, "cam");
NexDSButton gallina = NexDSButton(0, 11, "bt1");//BOTON GALLINA
NexDSButton codorniz = NexDSButton(0, 12, "bt2");// BOTON CODORNIZ
NexDSButton pato = NexDSButton(0, 13, "bt3");
NexDSButton arr = NexDSButton(1, 18, "bt01");
NexDSButton abj = NexDSButton(1, 19, "bt11");
NexDSButton prender = NexDSButton(1, 20, "bt21");
NexDSButton put1 = NexDSButton(3, 14, "put");
NexDSButton manual2 = NexDSButton(1, 21, "manual");
///--------------textos
NexText t0 = NexText(0, 1, "num");//NUMERO DE CEL QUE LLEGA EN PANTALLA
NexText t4 = NexText(0, 6, "celu");//NUMER DE CEL PUEBLICADO EN PANTALA
NexText t9 = NexText(0, 15, "aler");//MENSAJES ALERTA EN PANTALLA
NexText aire2 = NexText(1, 3, "aire");// MENSAJE DE AIRE
//////////////////////////////////////////____------------------------------------------------------numeros A RECIBIR 
NexNumber d1 = NexNumber(3, 2, "d1");
NexNumber d2 = NexNumber(3, 3, "d2");
NexNumber d3 = NexNumber(3, 4, "d3");
NexNumber tem1 = NexNumber(3, 5, "tem1");
NexNumber tem2 = NexNumber(3, 6, "tem2");
NexNumber tem3 = NexNumber(3, 7, "tem3");
NexNumber hum1 = NexNumber(3, 8, "hum1");
NexNumber hum2 = NexNumber(3, 9, "hum2");
NexNumber hum3 = NexNumber(3, 10, "hum3");
NexNumber vuel1 = NexNumber(3, 11, "vuel1");
NexNumber vuel2 = NexNumber(3, 12, "vuel2");
NexNumber vuel3 = NexNumber(3, 13, "vuel3");
NexText t50 = NexText(3, 15, "t50");
uint32_t num1,num2,num3,num4,num5,num6,num7,num8,num9,num10,num11,num12,num13;// NUMEROS PARA GUARDAR LAS VARIABLES


//------------------
//int niveldeagua=9;
float dias=0,dia1=0,dia2=0,reloj=0,reloj1=0;
/// creando datos pantalla nextion
byte addArray[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
 
char dataValue[9];

char llam[12]="0123456789";
//////////////////////////
bool gall=false,cor=false,pat=false,ban2=false;



 NexTouch *nex_listen_list[] = 
{
   
    &gallina,
    &codorniz,
    &pato,
    &com,
    &put1,
    NULL
};
//////fin pantalla
//------------------------------------------------------------------------------setup----------------------------------------------------------------------------------
void setup() {
  delay(1000);
  Serial.begin(9600);
  //mostar.begin(9600);
//digitalWrite(9, HIGH); // Descomentar para activar la alimentación de la tarjeta por Software
//sensor de presion
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale1.begin(LOADCELL_DOUT_PIN1, LOADCELL_SCK_PIN2);
// seconds, minutes, hours, day of the week, day of the month, month, year

/// configurar hora
//----------------------------------------------
// put your setup code 
  sensors1.begin();   //Se inicia el sensor 1
  sensors2.begin();   //Se inicia el sensor 2here, to run once:
  pinMode(sh, OUTPUT);
//////////////////
  pinMode(led, OUTPUT);   //defino el pin 13 definido como led como salida
  digitalWrite(led, LOW);
  pinMode(carga, OUTPUT);
  digitalWrite(carga,LOW);
  SIM900.begin(19200);//Arduino se comunica con el SIM900 a una velocidad de 19200bps
  delay(2000);//tiempo para que identifique la red el modulo sim 900
////////////////////led
  digitalWrite(sh,HIGH);
 //ultra sonido;
  pinMode(TRIG, OUTPUT);  // trigger como salida
  pinMode(ECO, INPUT);    // echo como entrada
//configiracion inicial  pantalla nextion de botones _____________________________

   
////CONFIGRACION PINES 
  pinMode(fcp, OUTPUT);
  pinMode(lampled, OUTPUT);
  pinMode(ven6, OUTPUT);
  
  digitalWrite(lampled,HIGH);
    delay(100);
  digitalWrite(ven6,HIGH);
    delay(100);
  pinMode(sn, INPUT_PULLUP);
 
  pinMode(ven1, OUTPUT);  
  pinMode(led1, OUTPUT); 
  pinMode(led2, OUTPUT);  
 

  pinMode(ven2, OUTPUT);  
  pinMode(ven3, OUTPUT);  
  pinMode(ven4, OUTPUT);  
  pinMode(ven5, OUTPUT);
 
  pinMode(damper, OUTPUT);  
  pinMode(r1, OUTPUT);  
  pinMode(r2, OUTPUT);
  pinMode(mb, OUTPUT);
  pinMode(hum, OUTPUT);
  pinMode(ca, OUTPUT); 
  pinMode(sh, OUTPUT);
  pinMode(fcp, INPUT);
 
  pinMode(lampuv, OUTPUT);
  //// ESTADO INICIAL PINES

  digitalWrite(sh,HIGH);
    digitalWrite(ven1,HIGH);
    delay(100);
   digitalWrite(ven2,HIGH);
   delay(100);
    digitalWrite(ven3,HIGH);
    delay(100);
    digitalWrite(ven4,HIGH);
    delay(100);
    digitalWrite(ven5,HIGH);
    delay(100);
    digitalWrite(damper,HIGH);
    delay(100);
    digitalWrite(r1,HIGH);
    delay(100);
    digitalWrite(r2,HIGH);
    delay(100);
    digitalWrite(mb,HIGH);
    delay(100);
    digitalWrite(hum,HIGH);
    delay(100);
    digitalWrite(ca,HIGH);
    delay(100);
    digitalWrite(lampuv,LOW);
    delay(100);
   digitalWrite(led1,HIGH);
   delay(100);
   digitalWrite(led2,HIGH);
    delay(100);
    //SIM900power();
   nexInit();
   nexSerial.begin(19200);
   //configiracion inicial botones en la pantalla-------------
   com.attachPop(comPopCallback, &com);
   pato.attachPop(patoPopCallback, &pato);
   gallina.attachPop(gallinaPopCallback, &gallina);
   codorniz.attachPop(codornizPopCallback, &codorniz);
   arr.attachPop(arrPopCallback, &arr);
   abj.attachPop(abjPopCallback, &abj);
   prender.attachPop(prenderPopCallback, &prender);
   put1.attachPop(put1PopCallback, &put1);
  
  
//___________________________________________RTc____________________
   rtc.begin();// iniciacion rtc
   Wire.begin();
   SIM900power();//iniciacion del modulo sim900 para poder enviar mensajes;
 
  EEPROM.get(13,dias);
  EEPROM.get(1,num2);
  EEPROM.get(2,num3);
  EEPROM.get(3,Setpoint);
  EEPROM.get(4,Setpoint1);
  EEPROM.get(5,Setpoint2);
  EEPROM.get(6,hd);
  EEPROM.get(7,hd1);
  EEPROM.get(8,hd2); 
  EEPROM.get(9,rot1); 
  EEPROM.get(10,rot2); 
  EEPROM.get(11,rot3);
  EEPROM.get(12,llam[0]);
  EEPROM.get(14,llam[1]);
  EEPROM.get(15,llam[2]);
  EEPROM.get(16,llam[3]);
  EEPROM.get(17,llam[4]);
  EEPROM.get(18,llam[5]);
  EEPROM.get(19,llam[6]);
  EEPROM.get(20,llam[7]);
  EEPROM.get(21,llam[8]);
  EEPROM.get(22,llam[9]);
  EEPROM.get(23,llam[10]);
  EEPROM.get(24,llam[11]);
  EEPROM.get(25,llam[12]);
}


//----------------------------------------------------------------------------------------SETUP------------------------------------------------

void SIM900power()
{
  pinMode(9, OUTPUT); 
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);
  delay(3000);
}
void graficartemperatura()
{
  // page2.show();
   sensors1.requestTemperatures();   //Se envía el comando para leer la temperatura
    t2= sensors1.getTempCByIndex(0); //Se obtiene la temperatura en ºC del sensor 1

     sensors2.requestTemperatures();   //Se envía el comando para leer la temperatura
    t3= sensors2.getTempCByIndex(0); //Se obtiene la temperatura en ºC del sensor 2
   tp=(t2+t3)/2;

  
   
  }
 void errorsensor(){//CALSE DEFINIDAD CUANDO LOS SENSORES DE HUMEDAD FALLAN
  digitalWrite(sh,LOW);
   delay(200);
   digitalWrite(sh,HIGH);
 }
//SENSOR HUMEDAD------------------------------------------------------------------------------------------------------------
void humedadgraficar()
{  
    uint32_t start = micros();
    int chk = DHT.read22(DHT22_PIN);

    uint32_t stop = micros();

    stat.total++;
    switch (chk)//VERIFICACACION FUNCIONAMIENDO SENSORES
    {
    case DHTLIB_OK:
        stat.ok++;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
          break;
    default:
        stat.unknown++;
        break;
    }
    
    
    hu=DHT.humidity-18;

    

    start = micros();
    chk = DHT.read22(DHT22_PIN1);
    stop = micros();

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
        break;
    default:
        stat.unknown++;
        break;
    }
     hu1=DHT.humidity-9;
    
 }//fin sensor de humedad -------------------------------------------------------------
//CALIDAD AIRE ---------------------------------------
void aire(){///revisar
 /* adc = analogRead(mq); 
  if( adc<500){
      calaire="Buena"; 
    }
  else{
      calaire="mala";
      envioMensaje(9); 
    }*/
   mostar.write(addArray, 9);
   mostar.readBytes(dataValue, 9);
   int resHigh = (int) dataValue[2];
   int resLow  = (int) dataValue[3];
   int pulse = (256*resHigh)+resLow;
   calaire = String(pulse)+" PPM";

   /*if (pulse>3000 ){se cambia 3000 por las PPm que soportan los huevos
     envioMensaje(9); 
     digitalWrite(damper,LOW);
   }*/
  
   //Serial.print(adc);
   
  }
//------------------------------------------------------------------------------------------------------------------------------------------------
///presion SE TOMAN LOS DATOS DE LOS SENSORES DE PRESION----------------------------------------------------------------------
void presiongraficar(){
     if (scale.is_ready() && scale1.is_ready() ){
    long reading = scale.read();
    long reading1 = scale.read();
    p1=reading*5*0.5/16777216; 
    p1=p1*1000;   
    p2=reading1*5*0.5/16777216;
    p2=p2*1000;
    pp=(p1+p2)/2;

    if(pp<1000){//
     envioMensaje(10);// MENSAJE DE ALERTA
    }
   } else {
    //Serial.println("HX711 not found.");
  }

  
  
   }
//---------------------------------------------------------------------------------------------
 /////ultra sonido SE TOMAN LOS DATOS DE-----------------------------------------------------------------------------------------------------------------------------------------------------
void ultrasonido ()
{
//page2.show();
  altura=38;
  digitalWrite(TRIG, HIGH);     // generacion del pulso a enviar
  delay(1);       // al pin conectado al trigger
  digitalWrite(TRIG, LOW);    // del sensor
  
  DURACION = pulseIn(ECO, HIGH);  // con funcion pulseIn se espera un pulso
            // alto en Echo
  DISTANCIA = DURACION / 58.2;    // distancia medida en centimetros
  niveldeagua=altura-DISTANCIA;
 // Serial.println(DISTANCIA);    // envio de valor de distancia por monitor //serial
  //Serial.println(niveldeagua);

   delay(200);  
  //CONTROL MOTO BOMBA -------------------------------------------------------------------
  if(digitalRead(sn==HIGH)){
    //serial.println("Nivel alto");
    digitalWrite(LED,HIGH);
    digitalWrite(mb,HIGH);
    
    }
   else if(digitalRead(sn==LOW)){
    //serial.println("Nivel BAJO");
    digitalWrite(mb,LOW);
    digitalWrite(LED,LOW);
    }

  if (niveldeagua<16){
    digitalWrite(mb,HIGH);
      t9.setText("llene el recipiente");
       envioMensaje(1);
    digitalWrite(LED,HIGH);
     
  }
  else if (niveldeagua>16 && digitalRead(sn)==LOW){
    digitalWrite(mb,LOW);
  
 
  }//FIN CONTROL--------------------------------------------------------------
   if((niveldeagua>100)||(niveldeagua<-100)){
    niveldeagua=aux1;// demora entre datos
   }else{
    aux1=niveldeagua;
   }
   
   niveldeagua1=3.1415*(12.5*12.5)*float(niveldeagua);
   niveldeagua1= niveldeagua1/1000; 
}
  ///////RTC----------------------------------------------------------------------------------------------------------------------------------------------------------
///////////terminar

/////////principio de control


 ///////////////7puertaa -------------------------------------------------------------------------------------------------------------------------------------
/////////777-----------------------------------------------------------------------------------------------------------------------------------------------------
void control(){
   if(tp>Setpoint){//CONTRO SI LA TEMPERATURA ES MAYOR ALA ASIGNADA
   digitalWrite(r1,HIGH);//LOGICA INVERTIDA SE PONEN LOS PINES EN 1 PARA APAGAR Y EN 0 PARA ACCIONAR
    digitalWrite(r2,HIGH);//APAGA R2
    digitalWrite(damper,LOW);//ENCIANDE DAMPER
    digitalWrite(ven5,LOW);//ENCIENDE VENTILADOR
    digitalWrite(ven4,LOW);//ENCIENDE VENTILADOR
    
    digitalWrite(ven1,HIGH);//
    digitalWrite(ven2,HIGH);
    digitalWrite(ven3,HIGH);
    digitalWrite(ven4,LOW);
  
    
    }
   if(tp<Setpoint-0.1){
    digitalWrite(r1,LOW);//ENCIENDDE RESISTENCIA 1
    digitalWrite(r2,LOW);//ENCIENDE R2
    digitalWrite(damper,HIGH);//APAGA DAMPER
    digitalWrite(ven5,HIGH);//APAGA VETILADOR
    digitalWrite(ven1,LOW);
    digitalWrite(ven2,LOW);
    digitalWrite(ven3,LOW);
    digitalWrite(ven4,HIGH);
    
   
    }
    if(tp>Setpoint+1){
      envioMensaje(8);
    }else if(tp<Setpoint-1){
      envioMensaje(7);
      
    }
   
    //----------------------
    //control humedad
   
    if(hu<0 || hu1<0){
     errorsensor();
    }else{
  
    hup=(hu+hu1)/2;
    if(hup>hd){
    digitalWrite(hum,HIGH);
    digitalWrite(ca,HIGH);
    }
    else if (hup<hd)
    {
    digitalWrite(hum,LOW);
    digitalWrite(ca,LOW);
    }
    if(hup+5>hd){
      envioMensaje(6);
    }else if(hup-10<hd){
      envioMensaje(5);
    }
    
    //---------------
    }
    }
  void motor(){
   DateTime now = rtc.now();
 
  if((mot1==mot2)&&(bandera==false)){
    bandera=true;
    mot1=now.hour(), DEC;
    mot2=now.hour(), DEC;
   Serial.print("yes////");
   
  }
   mot1=now.hour(), DEC;   
  //Serial.print("/"); 
  Serial.print("   ");       
  // Serial.print(myRTC.minutes);
  if((mot1==(mot2+(rot1+1)))||(mot1<mot2)) {
     envioMensaje(4);
    mot2=now.hour(), DEC;
  }
  if((mot1>mot2)&&(bandera==true)){
    bandera=false;
   
    
  }if(bandera==false){
     digitalWrite(led1,HIGH);
     digitalWrite(led2,LOW);
    delay(100);
    
  }else if(bandera==true){
    digitalWrite(led2,HIGH);
    digitalWrite(led1,LOW);
    delay(100);
    
  }
  
 }
 void envioMensaje(int b){

String op="";  
//op=String(valorsetpoint1);
 String a;

 //moscelu.setText(op1);
  a="AT+CMGS=\""+String(llam);
  a=a+"\"";
  

  
  //nversetpoint.setValue(valorsetpoint1);
    SIM900.print("AT+CMGF=1\r"); // comando AT para configurar el SIM900 en modo texto
  delay(200);
    SIM900.println(a);//reemplzar por el número a enviar el mensaje
  delay(200);
  switch (b){// MENSAJES ALERTA SE ESCOGE EL MENSAJE A ENVIAR
    case 1:
     SIM900.println("LLENE EL TANQUE NIVEL DE AGUA 16CM ");// Reemplzar por el texto a enviar
     delay(200);
     break;
    case 2:
     SIM900.println("INCUBACION TERMINADA");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 3:
     SIM900.println("LA PUERTA HA PERMANECIDO MUCHO TIEMPPO ABIERTA PUEDE AFECTAR LA INCUBACION ");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 4:
     SIM900.println("VOLTEANDO HUEVOS ");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 5:
     SIM900.println("HUMEDAD MUY BAJA");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 6:
     SIM900.println("HUMEDAD MUY ALTA ");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 7:
     SIM900.println("TEMPERATURA BAJA RIESGO DE PERDIDA DE HUEVO");// Reemplzar por el texto a enviar
     delay(200);
    break;
    case 8:
     SIM900.println("TEMPERATURA ALTA RIESGO DE PERDIDA DE HUEVO");// Reemplzar por el texto a enviar
     delay(200);
    break;
     case 9:
     SIM900.println("MALA CALIDAD DE AIRE");// Reemplzar por el texto a enviar
     delay(200);
    break;
     case 10:
     SIM900.println("PRESION ALTA");// Reemplzar por el texto a enviar
     delay(200);
    break;
    
    
     
  }
  
  //Finalizamos este comando con el caracter de sustitución (→) código Ascii 26 para el envio del SMS
  SIM900.println((char)26); 
  delay(200);
  SIM900.println();
}
void enviardatos(){
  
  
  char k[8];
 
 
  graficartemperatura();
  tep.setValue(tp);
  temp.setValue(tp);
  te1.setValue(t2);
  te2.setValue(t3);
  presiongraficar();
  pep.setValue(pp);
  pe1.setValue(p1);
  pe2.setValue(p2);
  humedadgraficar();
  hp.setValue(hup);
  h1.setValue(hu1);
  h2.setValue(hu);
  aire();
  calaire.toCharArray(k, 8);
  aire2.setText(k);
  
  naci.setValue(dias);
  ultrasonido();
  nivel.setValue(niveldeagua1);
  niel.setValue(niveldeagua1*5);
  t4.setText(llam);

 
 
  
   
  
  
  
}
void patoPopCallback(void *ptr)
{
    uint32_t dual_state;
}void put1PopCallback(void *ptr)
{
    uint32_t dual_state;
}
void arrPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void abjPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void prenderPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void codornizPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void gallinaPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void comPopCallback(void *ptr)
{
    uint32_t dual_state;
}
void recibirdatos(){
  uint32_t boton1,boton2,boton3,boton4,boton5,boton6,camt;
  
   gallina.getValue(&boton1);
   com.getValue(&camt);
   codorniz.getValue(&boton2);
   pato.getValue(&boton3);
   arr.getValue(&boton4);
   abj.getValue(&boton5);
   manual2.getValue(&boton6);
  
   if(camt==1){
     
    //guardamos lo que esta en t4 en el buffer
    t0.getText(llam,10);
    t4.setText(llam);
   
     EEPROM.put(12,llam[0]);
    EEPROM.put(14,llam[1]);
    EEPROM.put(15,llam[2]);
    EEPROM.put(16,llam[3]);
    EEPROM.put(17,llam[4]);
    EEPROM.put(18,llam[5]);
    EEPROM.put(19,llam[6]);
    EEPROM.put(20,llam[7]);
    EEPROM.put(21,llam[8]);
    EEPROM.put(22,llam[9]);
    EEPROM.put(23,llam[10]);
    EEPROM.put(24,llam[11]);
    EEPROM.put(25,llam[12]);
    
    
   }else {
    t4.setText(llam);
   }  
   
    if((boton1==1)&&(boton3==1)) 
    {
        t9.setText("elija solo una especie");
    }
    else if((boton2==1)&&(boton1==1))
    {
         t9.setText("elija solo una especie");
    }
    else if((boton2==1)&&(boton3==1))
    {
        t9.setText("elija solo una especie");
    }
    else if((boton2==1)&&(boton1==0)&&(boton3==0))
    {
      if(dias<1){
        if((gall==true)||(cor==true)||(pat==true)){
          t9.setText("Proceso terminado");
          gall=false;
          cor=false;
          pat=false;
          
        }else{
          cor=true;
          dias=18;
          Setpoint=37.7;
          Setpoint1=37.7;
          Setpoint2=37.7;
          hd=70.0;
          hd1=90.0;
          hd2=90.0;
          num2=3;
          num3=0;
          rot1=1;
          rot2=1;
          rot3=1;
          t9.setText("prosiga a la siguiente paginac");
           EEPROM.put(13,dias);
           EEPROM.put(1,num2);
           EEPROM.put(2,num3);
           EEPROM.put(3,Setpoint);
           EEPROM.put(4,Setpoint1);
           EEPROM.put(5,Setpoint2);
           EEPROM.put(6,hd);
           EEPROM.put(7,hd1);
           EEPROM.put(8,hd2); 
           EEPROM.put(9,rot1); 
           EEPROM.put(10,rot2); 
           EEPROM.put(11,rot3); 
        }
      }else{
        t9.setText("ya hay un proceso de incubacion");
      }
        
         
    } 
    else if((boton2==0)&&(boton1==0)&&(boton3==1))
    {
      if(dias<1){
        if((gall==true)||(cor==true)||(pat==true)){
          t9.setText("Proceso terminado");
          gall=false;
          cor=false;
          pat=false;
          num2=3;
          num3=0;
          
        }else{
          pat=true;
          dias=28;
          Setpoint=37.7;
          hd=70.0;
          hd1=90.0;
          hd2=90.0;
          Setpoint1=37.7;
          Setpoint2=37.7;
          rot1=1;
          rot2=1;
          rot3=1;
          t9.setText("prosiga a la siguiente paginap");
          num2=3;
          num3=0;
           EEPROM.put(13,dias);
           EEPROM.put(1,num2);
           EEPROM.put(2,num3);
           EEPROM.put(3,Setpoint);
           EEPROM.put(4,Setpoint1);
           EEPROM.put(5,Setpoint2);
           EEPROM.put(6,hd);
           EEPROM.put(7,hd1);
           EEPROM.put(8,hd2); 
           EEPROM.put(9,rot1); 
           EEPROM.put(10,rot2); 
           EEPROM.put(11,rot3); 
        }
      }else{
        t9.setText("ya hay un proceso de incubaciong");
      }
        
    }else if((boton2==0)&&(boton1==1)&&(boton3==0))
    {
      if(dias<1){
        if((gall==true)||(cor==true)||(pat==true)){
          t9.setText("Proceso terminado");
          gall=false;
          cor=false;
          pat=false;
          
        }else{
          gall=true;
          dias=20;
          Setpoint=37.7;
          hd=70.0;
          hd1=90.0;
          hd2=90.0;
          rot1=1;
          rot2=1;
          rot3=1;
          Setpoint1=37.7;
          Setpoint2=37.7;
          t9.setText("prosiga a la siguiente pagina");
          num2=3;
          num3=0;
           EEPROM.put(13,dias);
           EEPROM.put(1,num2);
           EEPROM.put(2,num3);
           EEPROM.put(3,Setpoint);
           EEPROM.put(4,Setpoint1);
           EEPROM.put(5,Setpoint2);
           EEPROM.put(6,hd);
           EEPROM.put(7,hd1);
           EEPROM.put(8,hd2); 
           EEPROM.put(9,rot1); 
           EEPROM.put(10,rot2); 
           EEPROM.put(11,rot3); 
        }
      }else{
        t9.setText("ya hay un proceso de incubacion");
      }
        
         
    }

    if(boton6==1){
    if((boton4==1)&&(boton5==0)){
      digitalWrite(led1,HIGH);
      digitalWrite(led2,LOW);
      delay(100);
    
    }else if((boton5==1)&&(boton4==0)){
      digitalWrite(led2,HIGH);
      digitalWrite(led1,LOW);
      delay(100);
    }else{
      digitalWrite(led2,HIGH);
      digitalWrite(led1,HIGH);
      delay(100);
     
      
      }
      }else{
        motor();
      }
  
      
   
}

void manual(){
  uint32_t boton1;

  put1.getValue(&boton1);

  if((boton1==1)&&(dias<1)){
     d1.getValue(&num1);
     d2.getValue(&num2);
     d3.getValue(&num3);
     dias=float(num1+num2+num3);
     hum1.getValue(&num4);
     hd=float(num4);
     hum2.getValue(&num5);
     hd1=float(num5);
     hum3.getValue(&num6);
     hd2=float(num6);
     vuel1.getValue(&num7);
     rot1=float(24/num7);
     vuel2.getValue(&num8);
     rot2=float(24/num8);
     vuel3.getValue(&num9);
     rot3=float(24/num9);
     tem1.getValue(&num10);
     tem2.getValue(&num11);
     tem3.getValue(&num12);
     Setpoint=float(num10);
     Setpoint1=float(num11);
     Setpoint2=float(num11);
      EEPROM.put(13,dias);
      EEPROM.put(1,num2);
      EEPROM.put(2,num3);
      EEPROM.put(3,Setpoint);
      EEPROM.put(4,Setpoint1);
      EEPROM.put(5,Setpoint2);
      EEPROM.put(6,hd);
      EEPROM.put(7,hd1);
      EEPROM.put(8,hd2); 
      EEPROM.put(9,rot1); 
      EEPROM.put(10,rot2); 
      EEPROM.put(11,rot3); 
     
     
  }else{
    if(dias>1){
    t50.setText("ya hay un proceso de incubacion");
    } 
  }
  
}

void dias23(){
  DateTime now = rtc.now();
  if(dias>=1){
    if(dias<(num2+num3)){
      hd=hd1;
      Setpoint=Setpoint1;
      rot1=rot2;
          
    }
    if(dias<num3){
      hd=hd2;
      Setpoint=Setpoint2;
      rot1=rot3;
    }
    if((dia1==dia2)&&(ban2==false)){
      ban2=true;
      dia1=now.day(),DEC;
      dia2=now.day(),DEC;
      EEPROM.put(13,dias);
      EEPROM.put(1,num2);
      EEPROM.put(2,num3);
      EEPROM.put(3,Setpoint);
      EEPROM.put(4,Setpoint1);
      EEPROM.put(5,Setpoint2);
      EEPROM.put(6,hd);
      EEPROM.put(7,hd1);
      EEPROM.put(8,hd2); 
      EEPROM.put(9,rot1); 
      EEPROM.put(10,rot2); 
      EEPROM.put(11,rot3); 
    }else if((dia1!=dia2)&&(ban2==true)){
      dias=dias-1;
      dia2=now.day(),DEC;
    }
    dia1=now.day(),DEC;
  } if(dias==1){
     envioMensaje(2);
  }
}



   


void puerta(){
    uint32_t boton6;
    prender.getValue(&boton6);
    DateTime now = rtc.now();
  if(boton6==1){
      digitalWrite(lampled,LOW); 
    }else if(digitalRead(fcp)==0){
       
       digitalWrite(lampled,LOW);
       
       digitalWrite(r1,HIGH);
       digitalWrite(r2,HIGH);
       digitalWrite(damper,HIGH);
       digitalWrite(ven5,HIGH);
       digitalWrite(ven4,HIGH);
       digitalWrite(hum,HIGH);
       digitalWrite(ca,HIGH);
       digitalWrite(ven1,HIGH);
       digitalWrite(ven2,HIGH);
       digitalWrite(ven3,HIGH);
       digitalWrite(ven4,HIGH);
         
    
    }
      else {
        
      digitalWrite(lampled,HIGH);
    //digitalWrite(ven6,1); 
      control();   
    }
      
     
    

}
  


   

void loop() {
 
bool ha=false;
   //dias23();   enviardatos();
   puerta();
  delay(100);
  recibirdatos();
  dias23();
  delay(100);
  manual();
 delay(100);
 enviardatos();
      
     
// Delay so the program doesn't print non-stop 
   q='5'+String(t2)+','+String(t3)+','+String(tp)+','+String(hu)+','+String(hu1)+','+String(hup)+','+String(niveldeagua1)+','+String(p1)+','+String(p2)+','+String(pp);                                                       //| 
   Serial.print(q);
  //|                                    
 
} 
