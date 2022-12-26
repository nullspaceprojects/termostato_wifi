/*********************  ESP8688 CLIENT 1 NODO REMOTO  ********************************/

#include <NullSpaceLib.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h> //configuratore: https://arduinojson.org/v6/assistant/#/step1
//DHT sensor library:1.4.4
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h> //Adafruit RTClib per DS3231 e DateTime
#include <ArduinoWebsockets.h> //Downloading ArduinoWebsockets@0.5.3

#define REMOTE_NODE_ID 1


#define DHTPIN 2      // Digital pin connected to the DHT sensor GPIO2=D4
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t g_dht_event;
TimerC timer_sampling_temp_hum;
uint32_t sampling_time_temp_hum;

const char* ssid = "SSID"; //Enter SSID
const char* password = "PASSWORD"; //Enter Password
const char* websockets_server_host = "192.168.0.1"; //Enter server adress
const uint16_t websockets_server_port = 8080; // Enter server port

class DataHolder
{
  public:
    DataHolder(){}
    float node_temperature;
    float node_humidity;
};
DataHolder g_DataHolder;

//WEBSOCKET CLIENT
using namespace websockets;
TimerC timer_websocket_client_send;
#define sampling_time_websocket_client_send 1000 //ms
TimerC timer_websocket_client_rcv;
#define sampling_time_websocket_client_rcv 500 //ms
WebsocketsClient client;

void ConnectWiFi()
{
  //CONNESSIONE AL WIFI
  WiFi.begin(ssid, password);
  //TENTATIVI DI CONNESSIONE
  int counter = 0;
  while (!WiFi.isConnected()) 
  {
    delay(200);
    Serial.println("WiFi Non Connesso...Aspetta");    
    if (++counter > 100)
    {
      //SE NON SI CONNETTE DOPO 100 TENTATIVI, RESETTA ESP 
      ESP.restart();
    }
     
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());   //You can get IP address assigned to ESP
}

void setup() {
  Serial.begin(115200);
  ConnectWiFi();
  WiFi.setAutoReconnect(true);

  // Initialize device.
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  sampling_time_temp_hum = sensor.min_delay / 1000; //ms

  connectWSClientToServer();

  timer_sampling_temp_hum.start();
  timer_websocket_client_send.start();
  timer_websocket_client_rcv.start();

}

void loop() 
{
  bool ok = read_temp_hum_node_station(g_dht_event);
  if(ok)
  {
    clientWSSendDataToServer();
  }

  clientCheckForIncomingMsgs();
}


bool read_temp_hum_node_station(sensors_event_t& dht_event)
{
  if(timer_sampling_temp_hum.getET()>=sampling_time_temp_hum)
  {
    //digitalWrite(PIN_RELE,!digitalRead(PIN_RELE));
    timer_sampling_temp_hum.reset();
    dht.temperature().getEvent(&dht_event);
    if (isnan(dht_event.temperature)) {
      //Serial.println(F("Error reading temperature!"));
      return false;
    }
    else {
      g_DataHolder.node_temperature = dht_event.temperature;

      //Serial.print(F("Temperature: "));
      //Serial.print(g_dht_event.temperature);
      //Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&dht_event);
    if (isnan(dht_event.relative_humidity)) {
      //Serial.println(F("Error reading humidity!"));
      return false;
    }
    else {
      g_DataHolder.node_humidity = dht_event.relative_humidity;
      //Serial.print(F("Humidity: "));
      //Serial.print(g_dht_event.relative_humidity);
      //Serial.println(F("%"));      
    }
  }
  return true;
}

void connectWSClientToServer()
{
    // try to connect to Websockets server
    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
    if(connected) {
        Serial.println("Connecetd!");
        //client.send("Hello Server");
    } else {
        Serial.println("Not Connected!");
    }
    
    // run callback when messages are received
    client.onMessage([&](WebsocketsMessage message) {
        Serial.print("Got Message: ");
        Serial.println(message.data());
    });
}

void clientWSSendDataToServer()
{
  if(timer_websocket_client_send.getET() >= sampling_time_websocket_client_send)
  {
    timer_websocket_client_send.reset();
    //send data over websocket to server
    //TODO: BETTER TO USE JSON
    //I USE MY COSTUM STRING PROTOCOL
    char msg[20];
    sprintf(msg,"!%d,%s,%d", REMOTE_NODE_ID,String(g_DataHolder.node_temperature,1),static_cast<int>(g_DataHolder.node_humidity));
    client.send(msg);
    
  }
}

void clientCheckForIncomingMsgs()
{
  if(timer_websocket_client_rcv.getET() >= sampling_time_websocket_client_rcv)
  {
    timer_websocket_client_rcv.reset();
    // let the websockets client check for incoming messages
    if(client.available()) {
        client.poll();
    }
  }
}
