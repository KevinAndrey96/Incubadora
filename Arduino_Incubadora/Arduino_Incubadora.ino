#include <SPI.h>


#include "EEPROM.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include "Nextion.h"

#include <virtuabotixRTC.h>   
#include "HX711.h"

#include <SoftwareSerial.h>
SoftwareSerial SIM900(10, 11);
dht DHT;
#define pin_temperatura     3
#define carga              13
#define led                12
float Setpoint=29.0,hd=60.0;
String calaire="";

//inicio definicion de actuadores
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
  #define sn 33 //Sensor de nivel de agua en el humidificador
  #define fcp 34 // final de carrera de la puerta
  #define sh 46 //pin de conmutacion de sensores de humedad
  #define lampled 47 //lampara led
  #define lampuv 48 //lampara UV
  #define mq A0 //sensonr de calidad de aire
//Fin definicion de actuadores

OneWire comuniacion(pin_temperatura);
DallasTemperature ds18b20(&comuniacion);
///////////////   ultrasonido 

//--------------------------------------------Variables de temperatura
float t3,t2,tp;
OneWire ourWire1(50);                //Se establece el pin 2  como bus OneWire
OneWire ourWire2(52);                //Se establece el pin 3  como bus OneWire
DallasTemperature sensors1(&ourWire1); //Se declara una variable u objeto para nuestro sensor1
DallasTemperature sensors2(&ourWire2); //Se declara una variable u objeto para nuestro sensor2
//--------------------------------------------Fin variables temperatura

//-------------------------------------------humedad
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
//----------------------------
//------------------------------------------//rtc
virtuabotixRTC myRTC(36,37,38);
#define led1 39 //giro del motor en un sentido
#define led2 40 //giro del motor en sentido contrario
#define led3 41 //Indicador de funcionamiento
#define in 42  //Bot+on de arranque
int flag=0,d,h,m=-1,s,aux=110,sg=0,as;
String q="";
//--------------------------------------------


//----------------------------------------ULTRASONIDO
//Pines y variables ultrasonido
int TRIG = 4;      // trigger en pin 10
int ECO = 5;      // echo en pin 9
int LED = 13;      // LED en pin 3
int DURACION;
int DISTANCIA;
int altura;
int niveldeagua;
//Fin pines y variables ultrasonido

////////////////

float temperatura = 0;
char celular[10];//celular
int  valorsetpoint1;

/////////////////////////////
uint32_t  diaincu;
uint32_t  humedadincu;
uint32_t  temperaturaincu;
uint32_t  vueltasincu;
/////////////////////////////

uint32_t  diaaxu;
uint32_t  humedadaxu;
uint32_t  temperaturaaxu;
uint32_t  vueltasaxu; 
/////////////////////////////////
uint32_t  diana;
uint32_t  humedadna;
uint32_t  temperaturana;
uint32_t  vueltana;
//////////////////////

uint32_t   nhax0 ;
uint32_t   nhax1;
uint32_t   nhax2 ; 
uint32_t   nhax3 ;
uint32_t   nhax4 ;
uint32_t   nhax5 ;

uint32_t   nhax01 ;
uint32_t   nhax11;
uint32_t   nhax21 ; 
uint32_t   nhax31 ;
uint32_t   nhax41 ;
uint32_t   nhax51 ;

/////////////////////////////
uint32_t   anax1 ;
uint32_t   anax2 ;
uint32_t   anax3;
uint32_t   anax4 ;
uint32_t   anax5 ;
uint32_t   anax6 ;
uint32_t   anax12 ;
uint32_t   anax22 ;
uint32_t   anax32;
uint32_t   anax42 ;
uint32_t   anax52 ;
uint32_t   anax62 ;

//////////////////////////
uint32_t  kiax1 ;
uint32_t  kiax2 ;
uint32_t  kiax3 ;
uint32_t  kiax4 ;
uint32_t  kiax5 ;
uint32_t  kiax6 ;
uint32_t  kiax13 ;
uint32_t  kiax23 ;
uint32_t  kiax33 ;
uint32_t  kiax43 ;
uint32_t  kiax53 ;
uint32_t  kiax63 ;
///////////////////////////
uint32_t  kfax1 ;
uint32_t  kfax2 ;
uint32_t  kfax3 ;
uint32_t  kfax4 ;
uint32_t  kfax5 ;
uint32_t  kfax6 ;
//---------
uint32_t  kfax14 ;
uint32_t  kfax24 ;
uint32_t  kfax34 ;
uint32_t  kfax44 ;
uint32_t  kfax54 ;
uint32_t  kfax64 ;
//////////////////////////
uint32_t efax1;
uint32_t efax2;
uint32_t efax3;
uint32_t efax4;
uint32_t efax5;
uint32_t efax6;
uint32_t efax15;
uint32_t efax25;
uint32_t efax35;
uint32_t efax45;
uint32_t efax55;
uint32_t efax65;
/////////////////////
uint32_t ktmax1;
uint32_t ktmax2;
uint32_t ktmax3;
uint32_t ktmax4;
uint32_t ktmax5;
uint32_t ktmax6;
uint32_t ktmax16;
uint32_t ktmax26;
uint32_t ktmax36;
uint32_t ktmax46;
uint32_t ktmax56;
uint32_t ktmax66;


/*NexDSButton gallina= NexDSButton(0, 13, "gallina");
NexDSButton personalizado = NexDSButton(0,14, "perso");
NexDSButton codorniz = NexDSButton(0,16, "codorniz");
NexDSButton pato = NexDSButton(0,20, "pato");
*/
/////////////////////////
//NexButton aceptar          = NexButton(0,3 , "aceptar");

//NexNumber nsetpoint          = NexNumber(0, 2, "nsetpoint");
NexText   celu = NexText(0,23,"celular1");
NexText   moscelu = NexText(0,22,"t3");
//NexNumber nversetpoint      = NexNumber(0, 1, "nversetpoint");

/////////////////////////////////////////////////////////declaracion incubadora
NexNumber diainc          = NexNumber(0, 4, "diai");
NexNumber tempinc         = NexNumber(0,5, "tempi");
NexNumber humeinc         = NexNumber(0, 6, "humei");
NexNumber vueltaini        = NexNumber(0, 21, "vueltai");
//////////////////////////////////////////////////////////AUXILIAR
NexNumber diaax         = NexNumber(0,10 , "diasi");
NexNumber tempax       = NexNumber(0,11 , "tempsi");
NexNumber humeax        = NexNumber(0, 12, "humesi");
NexNumber vueltaax        = NexNumber(0, 22, "vueltas");
/////////////////////////////////////////////////////////////NACIMINET0

NexNumber dianaci     = NexNumber(0, 7, "dian");
NexNumber tempnaci    = NexNumber(0,8, "tempn");
NexNumber humenaci     = NexNumber(0, 9, "humen");
NexNumber vueltanaci     = NexNumber(0, 23, "vueltan");

/////////////////////////////////////////////////////////nacimineto declaracion

//////////////////////////////////////////////////////////////////page 2 


//////////////// ///////////////////////////////////  temp 
NexNumber ts1    = NexNumber(2, 15, "t1");
NexNumber ts2     = NexNumber(2, 16, "t2");
NexNumber ntemperatura     = NexNumber(2, 2, "temperatura");
NexProgressBar jtermometro = NexProgressBar(2,5, "jtemperatura");

////////////////////////////////////////////////////// humedad 
NexNumber humedad     = NexNumber(2, 3, "humedad");
NexNumber hs1    = NexNumber(2, 17, "h1");
NexNumber hs2     = NexNumber(2, 18, "h2");
///////////////////////////////////////////////////////////////


NexText diastrascurrido    = NexText(2, 7, "dias");
NexText  calidadaire= NexText(2,23, "ca");
/////////////////////////////////////////////////////////
NexNumber presion    = NexNumber(2, 4, "presion");
NexProgressBar jpresion = NexProgressBar(2,6,"jpresion");
NexNumber ps1    = NexNumber(2, 19, "p1");
NexNumber ps2     = NexNumber(2, 20, "p2");
////////////////////////////////////////////////////////////////////////

NexNumber tanque    = NexNumber(2, 2, "tanque");
NexProgressBar jtanque = NexProgressBar(2,8, "jtanque");
NexText tmensaje           = NexText(2,10, "tmensaje");
//////////////////////////////////////////////////////////////////////
//NexText tmensaje           = NexText(2,10, "tmensaje");
NexNumber  horaactual=NexNumber(2,12,"horaactual");
NexNumber  minutosactual=NexNumber(2,13,"minutosactual");
/////////////////////////////////////////////////boton page 2 
/*
NexDSButton led50w= NexDSButton(2, 21, "led50w");
NexDSButton pausa = NexDSButton(2,22, "pausa");
//NexDSButton parar  = NexDSButton(2,22, "pausa");
*/
////////////////////////////////////////////page 3 




//////////////////////////////////////huevos 
NexNumber nh1     = NexNumber(3, 2, "n0");
NexNumber nh2     = NexNumber(3, 3, "n1");
NexNumber nh3     = NexNumber(3, 4, "n2");
NexNumber nh4     = NexNumber(3, 5, "n3");
NexNumber nh5     = NexNumber(3, 6, "n4");
NexNumber nh6     = NexNumber(3, 7, "n5");

NexNumber nhm1     = NexNumber(3, 8, "n6");
NexNumber nhm2     = NexNumber(3, 9, "n7");
NexNumber nhm3     = NexNumber(3, 10, "n8");
NexNumber nhm4     = NexNumber(3, 11, "n9");
NexNumber nhm5    = NexNumber(3, 12, "n10");
NexNumber nhm6     = NexNumber(3, 13, "n11");

////////////////////////////////////// aves nacidas



NexNumber an1     = NexNumber(3, 14, "n12");
NexNumber an2     = NexNumber(3, 15, "n13");
NexNumber an3     = NexNumber(3, 16, "n14");
NexNumber an4     = NexNumber(3, 17, "n15");
NexNumber an5     = NexNumber(3, 18, "n16");
NexNumber an6     = NexNumber(3, 19, "n17");



NexNumber anm1     = NexNumber(3, 20, "n18");
NexNumber anm2    = NexNumber(3, 21, "n19");
NexNumber anm3     = NexNumber(3, 22, "n20");
NexNumber anm4     = NexNumber(3, 23, "n21");
NexNumber anm5     = NexNumber(3, 24, "n22");
NexNumber anm6     = NexNumber(3, 25, "n23");




///////////////////////////////////////////////


/////////////////////kwiniciales
NexNumber ki1     = NexNumber(3, 26, "n24");
NexNumber ki2    = NexNumber(3, 27, "n25");
NexNumber ki3     = NexNumber(3, 28, "n26");
NexNumber ki4     = NexNumber(3, 29, "n27");
NexNumber ki5     = NexNumber(3, 30, "n28");
NexNumber ki6     = NexNumber(3, 31, "n29");

NexNumber kim1     = NexNumber(3, 32, "n30");
NexNumber kim2    = NexNumber(3, 33, "n31");
NexNumber kim3     = NexNumber(3, 34, "n32");
NexNumber kim4     = NexNumber(3, 35, "n33");
NexNumber kim5     = NexNumber(3, 36, "n34");
NexNumber kim6     = NexNumber(3, 37, "n35");


///////////////////////kwfinales 
NexNumber kf1     = NexNumber(3, 38, "n36");
NexNumber kf2    = NexNumber(3, 39, "n37");
NexNumber kf3     = NexNumber(3, 40, "n38");
NexNumber kf4     = NexNumber(3, 41, "n39");
NexNumber kf5     = NexNumber(3, 42, "n40");
NexNumber kf6     = NexNumber(3, 43, "n41");

NexNumber kfm1     = NexNumber(3, 44, "n42");
NexNumber kfm2     = NexNumber(3, 45, "n43");
NexNumber kfm3     = NexNumber(3, 46, "n44");
NexNumber kfm4     = NexNumber(3, 47, "n45");
NexNumber kfm5     = NexNumber(3, 48, "n46");
NexNumber kfm6     = NexNumber(3, 49, "n47");

//////////////////////////fecha 
NexText tf1  = NexText(3,68, "t6");
NexText tf2  = NexText(3,70, "t8");
NexText tf3  = NexText(3,72, "t10");
NexText tf4  = NexText(3,74, "t12");
NexText tf5  = NexText(3,76, "t14");
NexText tf6  = NexText(3,78, "t16");

NexText tfm1  = NexText(3,69, "t7");
NexText tfm2 = NexText(3,71, "t9");
NexText tfm3 = NexText(3,73, "t11");
NexText tfm4  = NexText(3,75, "t13");
NexText tfm5  = NexText(3,77, "t15");
//NexText tfm6  = NexText(3,79, "t17");

/////////////////////////////////////////////

////////////////////////////////////////eficiencia

NexNumber ef1      = NexNumber(3, 50, "n48");
NexNumber ef2      = NexNumber(3, 51, "n49");
NexNumber ef3      = NexNumber(3, 52, "n50");
NexNumber ef4      = NexNumber(3, 53, "n51");
NexNumber ef5      = NexNumber(3, 54, "n52");
NexNumber ef6      = NexNumber(3, 55, "n53");


////////////////////////////////////////// total de kw

NexNumber ktm1     = NexNumber(3, 56, "n54");
NexNumber ktm2     = NexNumber(3, 57, "n55");
NexNumber ktm3     = NexNumber(3, 58, "n56");
NexNumber ktm4     = NexNumber(3, 59, "n57");
NexNumber ktm5     = NexNumber(3, 60, "n58");
NexNumber ktm6     = NexNumber(3, 61, "n59");


//-----------------------------------------paginas 
/*
NexPage page1 = NexPage(0, 0, "config");
NexPage page2 = NexPage(2, 0, "sensores");
NexPage page3= NexPage(3, 0, "ESTA");*/


/////////////////
/*NexTouch *nex_listen_list[] = 
{
    &gallina,&personalizado,&led50w,&pausa,&pato,&codorniz,
    NULL
};
*/



////////////
/*void LlamadoFuncionDualBoton(void *ptr);
void LlamadoFuncionDualpersonalizado(void *ptr);
void botonpato(void *ptr);
void botoncodorniz(void *ptr);
//--------------------------
void botonled50w(void *ptr);
void botonpausa(void *ptr);*/


void setup() {
  Serial.begin(115200);
 nexInit();
//-----------------------------------------------------------Sensores de presion
   scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale1.begin(LOADCELL_DOUT_PIN1, LOADCELL_SCK_PIN2);
//---------------------------------------------------------------------------
   pinMode(led, OUTPUT);   //defino el pin 13 definido como led como salida
   digitalWrite(led, LOW);
  pinMode(carga, OUTPUT);
  digitalWrite(carga,LOW);

//--------------------------------------------------------------------botones page 1 

  //------------------------------------------------------------------------SIM   
 digitalWrite(9, HIGH); // Descomentar para activar la alimentación de la tarjeta por Software
  delay(1000); 
 digitalWrite(9, LOW);
  delay(500);  
  SIM900.begin(19200);//Arduino se comunica con el SIM900 a una velocidad de 19200bps
  delay(20000);//Tiempo prudencial para el escudo inicie sesión de red con tu operador
  
//----------------------------------------------------------
 pinMode(TRIG, OUTPUT);  // trigger como salida
 pinMode(ECO, INPUT);    // echo como entrada

////////////

//inicio configuracion ds18b20
  sensors1.begin();   //Se inicia el sensor 1
  sensors2.begin();   //Se inicia el sensor 2
//Fin configuraion ds18b20

//Inicio configuracion pines de actuadores
  pinMode(sn, INPUT_PULLUP);
  pinMode(fcp, OUTPUT);
  pinMode(ven1, OUTPUT);  
  pinMode(ven2, OUTPUT);  
  pinMode(ven3, OUTPUT);  
  pinMode(ven4, OUTPUT);  
  pinMode(ven5, OUTPUT);
  pinMode(ven6, OUTPUT); 
  pinMode(damper, OUTPUT);  
  pinMode(r1, OUTPUT);  
  pinMode(r2, OUTPUT);
  pinMode(mb, OUTPUT);
  pinMode(hum, OUTPUT);
  pinMode(ca, OUTPUT); 
  pinMode(sh, OUTPUT);
  pinMode(fcp, INPUT);
  pinMode(lampled, OUTPUT);
  pinMode(lampuv, OUTPUT);
    
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
    digitalWrite(ven6,HIGH);
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
    digitalWrite(lampled,HIGH);
    delay(100);
    digitalWrite(lampuv,LOW);
    delay(100);
//Inicio configuracion pines de actuadores

}


void valoresperso(){
  
    //////////////////////////////////////incubacion valores 
 diainc.getValue(&diaincu);     
 tempinc.getValue(& humedadincu);          
 humeinc.getValue(& temperaturaincu); 
 vueltaini.getValue(&vueltasincu );                 
      
  ///////////////////////////////////// axuliar 
  diaax.getValue(&diaaxu);       
 tempax.getValue(&temperaturaaxu);        
 humeax.getValue(&humedadaxu);          
 vueltaax.getValue(&vueltasaxu );
  //////////////////////////////////////////////////////////nacimiento valores 
dianaci.getValue(&diana);       
 tempnaci.getValue(&humedadna);        
 humenaci.getValue(&temperaturana);          
vueltanaci.getValue(&vueltana);  
  }

  void envioMensaje() {
//  page1.show();
// nsetpoint.getValue(&valorsetpoint);

  celu.getText(celular,10);
int valorsetpoint = int(celular);
//Serial.print(valorsetpoint);
  EEPROM.write(39,valorsetpoint); 

//Aquí
//valorsetpoint1=EEPROM.read(39,valorsetpoint);
//


String op="";
op=String(valorsetpoint1);
char op1[10]="";
op.toCharArray(op1, 10);
 moscelu.setText(op1);


  
  //nversetpoint.setValue(valorsetpoint1);
    SIM900.print("AT+CMGF=1\r"); // comando AT para configurar el SIM900 en modo texto
  delay(200);
    SIM900.println("AT + CMGS = \" valorsetpoint1\"");//reemplzar por el número a enviar el mensaje
  delay(200);
  SIM900.println("LLENE EL TANQUE NIVEL DE AGUA 16CM ");// Reemplzar por el texto a enviar
  delay(200);
  //Finalizamos este comando con el caracter de sustitución (→) código Ascii 26 para el envio del SMS
  SIM900.println((char)26); 
  delay(200);
  SIM900.println();
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

void humedadgraficar()
{  //Serial.println("Type,\tStatus,\tHumidity (%),\tTemperature (C)");
    // READ DATA
    //Serial.print("DHT22_sup, \t");
   // page2.show();
    uint32_t start = micros();
    int chk = DHT.read22(DHT22_PIN);
    uint32_t stop = micros();

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
    //    Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
    //    Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
    //    Serial.print("Time out error,\t");
        break;
    default:
        stat.unknown++;
    //    Serial.print("Unknown error,\t");
        break;
    }
    // DISPLAY DATA
    //Serial.print(DHT.humidity, 2);
    //Serial.print(",\t");
    //Serial.print(DHT.temperature, 2);
    //Serial.print(",\t");
    //Serial.println();
    hu=DHT.humidity-16;

    //Inicio intento segundo sensor

//Serial.print("DHT22_inf, \t");

    start = micros();
    chk = DHT.read22(DHT22_PIN1);
    stop = micros();

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
    //    Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
    //    Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
    //    Serial.print("Time out error,\t");
        break;
    default:
        stat.unknown++;
    //    Serial.print("Unknown error,\t");
        break;
    }
    // DISPLAY DATA
    //Serial.print(DHT.humidity, 2);
    //Serial.print(",\t");
    //Serial.print(DHT.temperature, 2);
    //Serial.print(",\t");
    //Serial.println();
    hu1=DHT.humidity-4;
    
    //Fin segundo sensor

   
}
void aire(){
  adc = analogRead(mq); 
  if(adc>100 && adc<150){
      calaire="Buena"; 
    }
  else{
      calaire="mala"; 
    }
  }
  
void presiongraficar(){
 // page2.show();
     if (scale.is_ready() && scale1.is_ready() ){
    long reading = scale.read();
    long reading1 = scale.read();
    p1=reading*5*0.5/16777216;    
    p2=reading1*5*0.5/16777216;
   } else {
    Serial.println("HX711 not found.");
  }

  delay(1000);

   }


 void tiempoactual(){
    //page2.show();
    myRTC.updateTime(); 

 }

 void tiempo_transcurrido(){
//  page2.show();
  myRTC.updateTime();
  if(myRTC.minutes!=aux ){
    aux=myRTC.minutes;
//    Serial.println(q);
  }
   
  if(myRTC.minutes!=aux && flag==1){
    aux=myRTC.minutes;
    m++;
    if(sg==1){
      sg=0;
    }
    else{
      sg=1;
    }
    if(m==60){
      m=0;
      h++;
      if(h==24){
        h=0;
        d++;
      }
    }

    s=myRTC.seconds-as;
    String ti_trans=String(d)+'/'+String(h)+':'+String(m)+':'+String(s);
     diastrascurrido.setText("ti_trans");
    
  }
 }
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
  //serial.println(DISTANCIA);    // envio de valor de distancia por monitor //serial
  //serial.println(niveldeagua);

   delay(200);       // demora entre datos

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
      tmensaje.setText("llene el recipiente");
      envioMensaje();
//    digitalWrite(LED,HIGH);
    //serial.println("LLENE EL RECIPIENTE");
  }
  else if (niveldeagua>16 && digitalRead(sn)==LOW){
    digitalWrite(mb,LOW);
  }
 
  }

  void control(){
    //Control de temperatura
    if(tp>Setpoint){
    digitalWrite(r1,HIGH);
    digitalWrite(r2,HIGH);
    digitalWrite(damper,LOW);
    digitalWrite(ven5,LOW);
    }
  else if(tp<Setpoint-0.1){
    digitalWrite(r1,LOW);
    digitalWrite(r2,LOW);
    digitalWrite(damper,HIGH);
    digitalWrite(ven5,HIGH);
    }
    //----------------------
    //control humedad
    if(hu<0 || hu1<0){
    digitalWrite(sh,LOW);
    delay(500);
    digitalWrite(sh,HIGH);
    }
  else{
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
    //---------------
    
    }
  }

  void graficar() {
   ts1.setValue(t2);
   ts2.setValue(t3);
   ntemperatura.setValue(tp);
   jtermometro.setValue(tp + 20);
   tanque.setValue(niveldeagua);
   jtermometro.setValue(niveldeagua);
   horaactual.setValue(myRTC.hours);
   minutosactual.setValue(myRTC.minutes);
   pp=(p2+p1)/2;
   presion.setValue(pp);
   ps1.setValue(p1);
   ps2.setValue(p2);
   jpresion.setValue(pp+20); 
   humedad.setValue((hu1+hu)/2);
   hs1.setValue(hu);
   hs2.setValue(hu1);
   char k[8];
   if(calaire=="Buena"){
    
    calaire.toCharArray(k, 8);
       calidadaire.setText(k);
    }
    else{
          calaire.toCharArray(k, 8);
         calidadaire.setText(k);
      }

    }
  
 void puerta(){
  if(digitalRead(fcp)==0){
    digitalWrite(lampled,0);    
    digitalWrite(ven6,0);    
    }
  else {
    digitalWrite(lampled,1);
    digitalWrite(ven6,1);    
    }
  }

void pagina3(){
//page3.show();
  //////////////////////////////////////incubacion
 nh1.getValue(&nhax0);
 EEPROM.write(2,nhax0);
 EEPROM.write(2,nhax0);

nhax01= EEPROM.read(2);
 
 
 nhm1.setValue(nhax01) ;

   //--------------------------
 nh2.getValue(&nhax1);
 EEPROM.write(3,nhax1);
 
 nhax11=EEPROM.read(3);    
 
 
 nhm2.setValue(nhax11) ;
//-----------------------------
 nh3.getValue(&nhax2);
 EEPROM.write(4,nhax2);
 nhax21=EEPROM.read(4);    
 nhm3.setValue(nhax21) ;

 //-------------------------

 nh4.getValue(&nhax3);
 EEPROM.write(5,nhax3); 
 nhax31=EEPROM.read(5);    
 nhm4.setValue(nhax31) ;

 //----------------------
  nh5.getValue(&nhax4);
 EEPROM.write(6,nhax4);
 nhax41=EEPROM.read(6);    
 nhm5.setValue(nhax41) ;
 //-----------------------
 nh6.getValue(&nhax5);
 EEPROM.write(7,nhax5);  
 nhax51=EEPROM.read(7);   
 nhm6.setValue(nhax51) ;        
  //------------------------------------------nacimiento valores 
 an1.getValue(&anax1);
 EEPROM.write(8,anax1); 
 anax12=EEPROM.read(8);     
 anm1.setValue(anax12) ;
 //---------------------   
 an2.getValue(&anax2);
  EEPROM.write(9,anax2); 
 anax22=EEPROM.read(9);     
 anm2.setValue(anax22) ;
 //--------------------
 an3.getValue(&anax3); 
 EEPROM.write(10,anax3); 
 anax32=EEPROM.read(10);    
 anm3.setValue(anax32) ; 
// -------------
 an4.getValue(&anax4);  
  EEPROM.write(11,anax4); 
 anax42=EEPROM.read(11);   
  anm4.setValue(anax42) ;
  //------------
 an5.getValue(&anax5); 
  EEPROM.write(12,anax5); 
 anax52=EEPROM.read(12);    
 anm5.setValue(anax52) ; 
 //-----------
 an6.getValue(&anax6); 
 EEPROM.write(13,anax6); 
 anax62=EEPROM.read(13);    
 anm6.setValue(anax62) ;  

////////////////////////////////////////////////////////

  ki1.getValue(&kiax1);
  EEPROM.write(14,kiax1); 
  kiax13=EEPROM.read(14);     
  kim1.setValue(kiax13) ; 
  //------------------
  ki2.getValue(&kiax2);   
    EEPROM.write(15,kiax2); 
  kiax23=EEPROM.read(15); 
  kim2.setValue(kiax23) ;
  //------------------
  ki3.getValue(&kiax3);  
    EEPROM.write(16,kiax3); 
  kiax33=EEPROM.read(16);  
  kim3.setValue(kiax33) ; 
  //-----------------------
  ki4.getValue(&kiax4);
    EEPROM.write(17,kiax4); 
  kiax43=EEPROM.read(17);    
  kim4.setValue(kiax43) ; 
 //---------------------------
  ki5.getValue(&kiax5); 
    EEPROM.write(18,kiax5); 
  kiax53=EEPROM.read(18);   
  kim5.setValue(kiax53) ;
  //--------------------- 
  ki6.getValue(&kiax6);
  EEPROM.write(19,kiax6); 
  kiax63=EEPROM.read(19);    
  kim6.setValue(kiax63) ; 

///////////////////////////////////


  kf1.getValue(&kfax1); 
   EEPROM.write(20,kfax1); 
  kfax14=EEPROM.read(20);   
  kfm1.setValue(kfax14) ; 
  //------------------------
  kf2.getValue(&kfax2); 
     EEPROM.write(21,kfax2); 
  kfax24=EEPROM.read(21);
  kfm2.setValue(kfax24) ;
  //-----------------------
  kf3.getValue(&kfax3); 
     EEPROM.write(22,kfax3); 
  kfax34=EEPROM.read(22) ;  
  kfm3.setValue(kfax34) ; 
  //---------------------
  kf4.getValue(&kfax4);   
     EEPROM.write(23,kfax4); 
  kfax44=EEPROM.read(23); 
  kfm4.setValue(kfax44) ;
  //--------------------------- 
  kf5.getValue(&kfax5); 
     EEPROM.write(24,kfax5); 
  kfax54=EEPROM.read(24);   
  kfm5.setValue(kfax54) ;
  //--------------------------- 
  kf6.getValue(&kfax6);
  EEPROM.write(25,kfax6); 
  kfax64=EEPROM.read(25) ;   
  kfm6.setValue(kfax64) ; 
//----------------------------------------------------------EFICIENCIA 

   efax1= (100* anax12)/ nhax01 ;
   EEPROM.write(26,efax1); 
   efax15=EEPROM.read(26) ;
   ef1.setValue(efax15);
   //-------------------
   efax2=(100* anax22)/ nhax11  ;
   EEPROM.write(27,efax2); 
    efax25=EEPROM.read(27);
   ef2.setValue(efax25);
   //-----------------------
   efax3=(100* anax32)/ nhax21 ;
      EEPROM.write(28,efax3); 
 efax35=EEPROM.read(28);
   ef3.setValue(efax35);
//---------------------------
   efax4=(100* anax42)/ nhax31 ;
      EEPROM.write(29,efax4); 
  efax45=EEPROM.read(29);
   ef4.setValue(efax45);
   //------------------------
   efax5=(100* anax52)/ nhax41 ;
      EEPROM.write(30,efax5); 
 efax55=EEPROM.read(30) ;
   ef5.setValue(efax55);
   //--------------------------
   efax6=(100* anax12)/ nhax51 ;
      EEPROM.write(31,efax6); 
 efax65=EEPROM.read(31) ;
   ef6.setValue(efax65);

///////////////////////////////////////
   ktmax1=(100*kfax14)/ kiax13;
   EEPROM.write(32,  ktmax1); 
     ktmax16=EEPROM.read(32) ;
   ktm1.setValue(ktmax16);
    //--------------------------
   ktmax2=(100*kfax24)/ kiax23;
    EEPROM.write(33,  ktmax2); 
     ktmax26=EEPROM.read(33);
   ktm2.setValue(ktmax26);
    //--------------------------
   ktmax3=(100*kfax34)/ kiax33;
     EEPROM.write(34,  ktmax3); 
     ktmax36=EEPROM.read(34) ;
       ktm3.setValue(ktmax36);
        //--------------------------
        ktmax4=(100*kfax44)/ kiax43;
            EEPROM.write(35,  ktmax4); 
     ktmax46=EEPROM.read(35) ;
   ktm4.setValue(ktmax46);
    //--------------------------
   ktmax5=(100*kfax54)/ kiax53;
          EEPROM.write(36,  ktmax5); 
     ktmax56=EEPROM.read(36) ;
   ktm5.setValue(ktmax56);
    //--------------------------
   ktmax6=(100*kfax54)/ kiax53;
          EEPROM.write(37,  ktmax6); 
     ktmax66=EEPROM.read(37) ;
   ktm6.setValue(ktmax66);

  }




void loop() {
 
// nexLoop(nex_listen_list);

unsigned long tiempo = millis();

 /*gallina.attachPop(LlamadoFuncionDualBoton, &gallina);
 personalizado.attachPop(LlamadoFuncionperzonalizado, &personalizado);
 codorniz.attachPop(botoncodorniz,&codorniz);
 pato.attachPop(botoncodorniz,&pato);
//---------------------------------------------------------------------------botones page 2 
led50w.attachPop(botonled50w,&led50w);
pausa.attachPop(botonpausa,&pausa);*/



if ( (unsigned long)(tiempo - t) >= intervalo)
  {
       Serial.println("Entra");
//   envioMensaje() ;
//   aire();
//   tiempo_transcurrido();
  // valoresperso();
   //graficartemperatura();
   //ultrasonido();
   //presiongraficar();
   //tiempoactual();
   //humedadgraficar();
   
    //t = tiempo;

  }
   //puerta();
   //pagina3();
   //control();
   //graficar();
//q=String(t2)+','+String(t3)+','+String(tp)+','+String(hu)+','+String(hu1)+','+String(hup)+','+String(niveldeagua)+','+String(p1)+','+String(p2)+','+String(pp);
//   Serial.println(q);

  

  digitalWrite(ven1,LOW);
  delay(500);
  digitalWrite(ven2,LOW);
  delay(500);
  digitalWrite(ven3,LOW);
  delay(500);
  digitalWrite(ven4,LOW);
  delay(500);

  
 
  //}

}

/*void botonpato(void *ptr){
uint32_t estadopato;
    /* Consigo el valor del estado del componente. 
    pato.getValue(&estadopato);
    
    if(estadopato) 
    {
     int p=1;//  Serial.print("encendido");
      // digitalWrite(aaa, HIGH);
    }
    else
    {
     Serial.print("apagado");
       //digitalWrite(carga, LOW);
    }
  
}

void botoncodor(void *ptr)
{
  
    uint32_t estadocodor;
    /* Consigo el valor del estado del componente. 
    codorniz.getValue(&estadocodor);
    
    if(estadocodor) 
    {
     int i=1;//  Serial.print("encendido");
      // digitalWrite(aaa, HIGH);
    }
    else
    {
Serial.print("apagado");
       //digitalWrite(carga, LOW);
    }
  }

void LlamadoFuncionDualBoton(void *ptr)
{
    uint32_t Estado_dual;
    /* Consigo el valor del estado del componente. 
    gallina.getValue(&Estado_dual);
    
    if(Estado_dual) 
    {
       Serial.print("encendido");
       digitalWrite(carga, HIGH);
    }
    else
    {
       Serial.print("apagado");
       digitalWrite(carga, LOW);
    }
}




void LlamadoFuncionperzonalizado(void *ptr)
{
    uint32_t Estado_dual1;
    /* Consigo el valor del estado del componente. 
    personalizado.getValue(&Estado_dual1);
    
    if(Estado_dual1) 
    {
       Serial.print("encendido");
       digitalWrite(led, HIGH);
    }
    else
    {
       Serial.print("apagado");
       digitalWrite(led, LOW);
    }
}


void botonled50w(void *ptr){
  
    uint32_t estadoled;
//    Consigo el valor del estado del componente
    led50w.getValue(&estadoled);
    
    if(estadoled) 
    {
      Serial.print("encendido");
    }
    else
    {
       Serial.print("apagado");
          }
  
   }


   void botonpausa(void *ptr){
  
    uint32_t estadopausa;
//    Consigo el valor del estado del componente.
    pausa.getValue(&estadopausa);
    
    if(estadopausa) 
    {
      Serial.print("encendido");
    }
    else
    {
       Serial.print("apagado");
          }
     }
     */
