#include <Arduino.h>
#include <WiFi.h>
#include "PubSubClient.h" //pio lib install "knolleary/PubSubClient" 

#define triggerPin 4      // pin voor de trigger
#define echoPin 2         // pin voor de echo
#define soundSpeed 343.0  // snelheid van het geluid (m/s)
const byte buzzer = 25;
const byte slot = 26;
// init variables
long echoTime = 0;
float distance = 0;

const byte druk = 19;
const byte led_gpio = 32; // pin waar de PWM pin van alle motors aangesloten wordt
const byte pin_links = 5; // pin waar de richting van de linkse motoren aanhangt
const byte pin_rechts = 18; // pin waar de richting van de rechtse motoren aanhangt
int brightness = 200; // snelheid waarmee de motor draait
const byte IR_RECEIVE_PIN = 12; //aansluiten pin remote
int code = 0;
int timeint = 30000;
int rssi[10];
int i = 0;
int besturing[4]= {0,1,2,3};

void shuffle(){
  int k = random(1,3);
  for (int d = 0 ; d<4;d++){
    besturing[d] = (besturing[d]+k)%4;
    Serial.println(besturing[d]);
  }
  Serial.println("shuffle");
}

char* ssid1 = "stralingslocatie";
char* password1 = "123456789";
char* ssid2 = "NETGEAR68";
char* password2 = "excitedtuba713";

#define MQTT_SERVER   "192.168.1.2"
#define MQTT_PORT     1883
IPAddress local_IP(192, 168, 1, 18);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); // optional
IPAddress secondaryDNS(8, 8, 4, 4);



WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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
  Serial.print("Afstand = ");
  Serial.print((String)distance);
  Serial.println(" cm");
  return distance;
}

void wifi1(){
  WiFi.disconnect();
  delay(100);
  Serial.println("verbinden met esp");
   if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid1,password1);
    Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("verbonden met esp");
  Serial.println(WiFi.localIP());
} 
void wifi2(int rssi){
  WiFi.disconnect();
  delay(100);
  Serial.println("verbinden met broker");
  WiFi.begin(ssid2,password2);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print("."); 
    
  }
  Serial.println();
  char cstr[16];
  int num = rssi;
  String str;
  str = String(num);
  str.toCharArray(cstr,16);
  Serial.println(cstr);
  Serial.println("ok2");
  Serial.println("ok2");
  client.connect("esp32auto");
  client.publish("rssiwaarde", cstr);
  Serial.println(cstr);
  
  Serial.print(client.state());
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
}

void setup() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  ledcSetup(0, 8000, 8);
  ledcAttachPin(led_gpio , 0);  
  Serial.begin(115200);
  pinMode(pin_links, OUTPUT);
  pinMode(pin_rechts, OUTPUT);
  ledcWrite(0,255);
  Serial.println("Start") ;
  Serial.println("verbinden met esp");
   if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid1,password1);
  Serial.print("Connecting");
  pinMode(triggerPin, OUTPUT);  // zet trigger pin als uitgang
  pinMode(echoPin, INPUT);      // zet echo pin als ingang
  pinMode(slot, OUTPUT); 
  pinMode(buzzer, OUTPUT); 
  digitalWrite(slot, HIGH);
  digitalWrite(buzzer ,LOW);
}

void loop() { 
  long now = millis();
  Serial.println(now);
  Serial.println(lastMsg);
  if (now - lastMsg > 10000)
  {
    lastMsg = now;
    shuffle();
  }
  
  if( digitalRead(13)==HIGH){   
    Serial.println("ontvangen signaal");
    if( digitalRead(12)==HIGH){
      code = besturing[0];
    }    
    else if( digitalRead(14)==HIGH){
      code = besturing[1];
    }
    else if( digitalRead(27)==HIGH){
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
  
  rssi[i] = WiFi.RSSI();
  i++;
  if(i == 9){
    int rss =0;    
    for(int d= 0; d<10;d++){
      rss =rss + rssi[d];
    }
    Serial.println(rss);
    rss= rss/(-10);
    if (getDistance()>10){
      wifi2(200);rss=200;
      while(getDistance()>10){
        digitalWrite(buzzer ,HIGH);
        delay(200);
        digitalWrite(buzzer ,LOW);
      }      
    }
    
    wifi2(rss);    
    if(rss == 200){
      for(int o = 0; o< 10 ; o++ ){
        digitalWrite(buzzer ,HIGH);
        delay(400);
        digitalWrite(buzzer ,LOW);
      }
    }
    if(rss < 30){
      if(digitalRead(druk)==HIGH){
      stop();
      delay(100);
      digitalWrite(slot, LOW);
      delay(2000);
      digitalWrite(slot, HIGH);}
    }    
    wifi1();
    i =0;
  }
  
}
