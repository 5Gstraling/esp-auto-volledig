#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
//afstandsbediening
const byte Aint = 27; 
const byte Ad1 = 26; 
const byte Ad2 = 25;  
const byte Ad3 = 33;  

// esp now

//uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xEE, 0x2E, 0xA8};// mac adress esp thuis led
//uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0x2A, 0xFB, 0xCC};// mac adress pcb ledbar
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0x2A, 0xFB, 0xF0};// mac adress pcbway
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {  
  int b;  
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

// afstandsensor
#define triggerPin 32  // pin voor de trigger
#define echoPin 2         // pin voor de echo
#define soundSpeed 343.0  // snelheid van het geluid (m/s)
int initialValue = 0;

// init variables
long echoTime = 0;
float distance = 0;

// buzzer
const byte buzzer = 16; 

//slot
const byte slot = 4;
int lastrssi;
const int opendeurrssi = 20;

// drukknop
const byte druk = 17; 

// motoren
 const byte led_gpio = 15; // pin waar de PWM pin van alle motors aangesloten wordt
 const byte pin_links = 13; // pin waar de richting van de linkse motoren aanhangt
 const byte pin_rechts = 14; // pin waar de richting van de rechtse motoren aanhangt
 int brightness = 200; // snelheid waarmee de motor draait

// tijd in seconde dat de besturing wisselt
int timeint = 30000;

// om de hoeveel waarden de rssi waarde wordt gepubliceerd
const int aantalWaarde= 40;

//beginwaarden
int code = 0;
int rssi;
int i = 0; //aantal van huidige waarde van rssi die aangepast wordt
int besturing[4]= {0,1,2,3};

// wifinetwerkgegevens stralingslocatie
char* ssid1 = "stralingslocatie";
char* password1 = "123456789";

// tijd sinds laatste shuffle
long lastMsg = 0;
char msg[50];


void shuffle(){
  int k = random(1,3);
   for (int d = 0 ; d<4;d++){
     besturing[d] = (besturing[d]+k)%4;
     //Serial.println(besturing[d]);
   }
   //Serial.println("shuffle");
 }



int getDistance(){
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
    // meet lengte van de echo puls
  echoTime = pulseInLong(echoPin, HIGH);
  // en bereken hiermee de afstand (in cm)
  // distance = float(echoTime) / 2 * (soundSpeed / 10000.0);
  // en geef weer op de seriele monitor
  //Serial.print("Afstand = ");
  //Serial.print((String)distance);
  //Serial.println(" cm");
  delay(200);
  return echoTime;
  
}

void setupDistance(){
  // initialValue = 0;
  // for( int nu = 0 ; nu < 10; nu++){
  //   initialValue =initialValue + getDistance();
  //   Serial.println(initialValue);
  //   delay(400);
  // }
  // initialValue = initialValue/10;

  initialValue =243;
  Serial.println("aftsand");
  Serial.println(getDistance());
}

void wifi1(){
  //WiFi.disconnect();
  delay(100);
  Serial.println("verbinden met esp");  
  WiFi.begin(ssid1,password1);
  Serial.print("Connecting");
  Serial.println(WiFi.macAddress());
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");    
  }
  Serial.println();
  Serial.print("verbonden met esp");
  Serial.println(WiFi.localIP());
} 

 void vooruit(){
   ledcWrite(0,brightness);
   digitalWrite(pin_links , HIGH);
   digitalWrite(pin_rechts , HIGH);
 }
 void achteruit(){
   ledcWrite(0,brightness);
   digitalWrite(pin_links , LOW);
   digitalWrite(pin_rechts , LOW);
 }
 void draailinks(){
   ledcWrite(0,brightness);
   digitalWrite(pin_links , LOW);
   digitalWrite(pin_rechts , HIGH);
 }
 void draairechts(){
   ledcWrite(0,brightness);
   digitalWrite(pin_links , HIGH);
   digitalWrite(pin_rechts , LOW);
 }
 void stop(){ledcWrite(0,255);
 //Serial.println("Stop") ;
 }


void setup() {
  WiFi.mode(WIFI_STA);
  // client.setServer(MQTT_SERVER, MQTT_PORT);
  ledcSetup(0, 25000, 8);
  ledcAttachPin(led_gpio , 0);  
  Serial.begin(115200);
  pinMode(pin_links, OUTPUT);
  pinMode(pin_rechts, OUTPUT);
  ledcWrite(0,255);
  Serial.println("Start") ;
  Serial.println("verbinden met esp");
  WiFi.begin(ssid1,password1);
  Serial.print("Connecting");
  pinMode(triggerPin, OUTPUT);  // zet trigger pin als uitgang
  pinMode(echoPin, INPUT);      // zet echo pin als ingang
  pinMode(slot, OUTPUT); 
  pinMode(buzzer, OUTPUT); 
  digitalWrite(slot, LOW);
  digitalWrite(buzzer ,LOW);
  pinMode(Aint , INPUT);
  pinMode(Ad1 , INPUT);
  pinMode(Ad2 , INPUT);
  pinMode(Ad3 , INPUT);
  i = 0;
  rssi = 0;
  
  lastrssi = 100;

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  setupDistance();
}

void loop() { 
   wisselen van de besturing
   long now = millis();
   //Serial.println(now);
   //Serial.println(lastMsg);
   if (now - lastMsg > 10000)
   {
     lastMsg = now;
     shuffle();
   }
  
   // besturen van de auto
   if( digitalRead(Aint)==HIGH){   
     //Serial.println("ontvangen signaal");
     if( digitalRead(Ad1)==HIGH){
       code = besturing[0];
     }    
     else if( digitalRead(Ad2)==HIGH){
       code = besturing[1];
     }
     else if( digitalRead(Ad3)==HIGH){
       code = besturing[2];
     }
     else{
       code = besturing[3];
     }
     switch (code) 
     {
       case 0:    
         Serial.println("vooruit");
         vooruit();
         break;
       case 1:    
         Serial.println("achteruit");
         achteruit();
         break;
       case 2:    
         Serial.println("draai links");
         draailinks();
         break;
       case 3:    
         Serial.println("draai rechts");
         draairechts();
         break;      
     } 
   } 
   else{
     stop();
   }
   sturen van de rssi waarde  
  rssi = rssi + WiFi.RSSI();
  delay(50);
  //Serial.println(rssi);
  i++;
  if(i == (aantalWaarde-1)){
    rssi= rssi/(-aantalWaarde);
    lastrssi = rssi;
    Serial.println(rssi);
    
    myData.b = rssi;
    

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  i=0;
  rssi = 0;
    //afstand bepalen initialValue is doorsnee 238
   stop();
   int abi = getDistance();
   Serial.println(abi);
   if (abi> 320){
     delay(150);
       if (getDistance()> 320){
       while(getDistance()> 310){
         digitalWrite(buzzer ,HIGH);        
       } 
       for(int o = 0; o< 40 ; o++ ){ //komt overeen met 42 seconden
         digitalWrite(buzzer ,HIGH);
         delay(50);
         digitalWrite(buzzer ,LOW);
         delay(50*o);
       }
     } }
  }
  //slot
  if (lastrssi <= opendeurrssi ){
    if(digitalRead(druk) == HIGH){
      digitalWrite(slot, HIGH);
      delay(2000);
      digitalWrite(slot, LOW);
    }
  }
  delay(100);
}
