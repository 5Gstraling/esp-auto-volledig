#include <Arduino.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include <esp_now.h>
//afstandsbediening
const byte Aint = 26;
const byte Ad1 = 27;
const byte Ad2 = 33;
const byte Ad3 = 25;

// esp now

//uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xEE, 0x2E, 0xA8};// mac adress esp thuis led
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0x2A, 0xFB, 0xCC};// mac adress pcb ledbar
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
#define triggerPin 15      // pin voor de trigger
#define echoPin 2         // pin voor de echo
#define soundSpeed 343.0  // snelheid van het geluid (m/s)

// init variables
long echoTime = 0;
float distance = 0;

// buzzer
const byte buzzer = 32;

//slot
const byte slot = 4;
int lastrssi;
const int opendeurrssi = 20;

// drukknop
const byte druk = 19;

// motoren
const byte led_gpio = 14; // pin waar de PWM pin van alle motors aangesloten wordt
const byte pin_links = 12; // pin waar de richting van de linkse motoren aanhangt
const byte pin_rechts = 13; // pin waar de richting van de rechtse motoren aanhangt
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

// wifinetwerkgegevens wifi broker
// char* ssid2 = "NETGEAR68";
// char* password2 = "excitedtuba713";

// mqtt broker
#define MQTT_SERVER   "192.168.1.2"
#define MQTT_PORT     1883
IPAddress local_IP(192, 168, 1, 122);// ip adres dat ik wil hebben
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);



WiFiClient espClient;
PubSubClient client(espClient);

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
  distance = float(echoTime) / 2 * (soundSpeed / 10000.0);
  // en geef weer op de seriele monitor
  //Serial.print("Afstand = ");
  //Serial.print((String)distance);
  //Serial.println(" cm");
  return distance;
  
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
  client.setServer(MQTT_SERVER, MQTT_PORT);
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
  WiFi.mode(WIFI_STA);
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
}

void loop() { 
  // wisselen van de besturing
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
  // sturen van de rssi waarde  
  rssi = rssi + WiFi.RSSI();
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
  }
  // slot
  if (lastrssi <= opendeurrssi ){
    if(digitalRead(druk) == HIGH){
      digitalWrite(slot, HIGH);
      delay(2000);
      digitalWrite(slot, LOW);
    }
  }
  // afstand bepalen
  if (getDistance()>10){
    delay(50);
    if (getDistance()>10){
      stop();
      while(getDistance()>10){
        digitalWrite(buzzer ,HIGH);
        delay(200);
        digitalWrite(buzzer ,LOW);
      } 
      for(int o = 0; o< 100 ; o++ ){
        digitalWrite(buzzer ,HIGH);
        delay(400);
        digitalWrite(buzzer ,LOW);
      }
    }}
}
