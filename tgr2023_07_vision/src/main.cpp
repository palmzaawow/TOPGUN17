#include "main.h"
#include "hw_camera.h"
// add header file of Edge Impulse firmware
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <nininininininin-project-1_inferencing.h> //wait
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <map>
#include<iostream>
// constants
#define TAG     "main"

#define BTN_PIN 0

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3
#define BMP_BUF_SIZE                             (EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE)

#define WIFI_SSID        "TGR17_2.4G" //ECC-Fighting
#define WIFI_PASSWORD    "" 
IPAddress ip(192, 168, 1, 58); 
IPAddress gateway(192, 168, 1, 1); 
IPAddress subnet(255, 255, 255, 0); 

bool gi = false;
const char *notf = "invalid";

const char *mqtt_broker = "192.168.1.2";
const char *topic = "TGR_17";
const char *topiss = "TGR_17/status";
const char *topicsud = "TGR_17/h2o/cmd";
const char *mqtt_username = "TGR_GROUP17";  //ex not use
const char *mqtt_password = "MQ405O"; // ex not use
const int mqtt_port = 1883;

uint8_t order = 0;
StaticJsonDocument<128> cmd_doc;

static char cmd_buf[128];

// static variables
static uint8_t *bmp_buf;

WiFiClient espClient;
PubSubClient client(espClient);

const size_t jsonDocumentSize = JSON_OBJECT_SIZE(6);

// Create a JSON document
StaticJsonDocument<jsonDocumentSize> jsonDocument;//Data_Label_Match

std::vector<std::map<std::string, float>> array_of_maps;

//Global value
float acc;

// static function prototypes
void print_memory(void);
void ei_prepare_feature(uint8_t *img_buf, signal_t *signal);
int ei_get_feature_callback(size_t offset, size_t length, float *out_ptr);
void ei_use_result(ei_impulse_result_t result);
void senddata(void);
void wificonnectSimple();
void mqtest();
void callbackk(char *topic, byte *payload, unsigned int length);
void initmap();
// initialize hardware
void setup() {
  
  initmap();
  Serial.begin(115200);
  //print_memory();
  pinMode(0, INPUT_PULLUP);
  wificonnectSimple();
  mqtest();
  hw_camera_init();
  bmp_buf = (uint8_t*)ps_malloc(BMP_BUF_SIZE);
  if (psramInit()) {
    ESP_LOGI(TAG, "PSRAM initialized");
  } else {
    ESP_LOGE(TAG, "PSRAM not available");
  }
}

// main loop
void loop() {
  static bool press_state = false;
  static uint32_t prev_millis = 0;
  client.loop();

  if(gi){

      uint32_t Tstart, elapsed_time;
      uint32_t width, height;

      prev_millis = millis();
      Tstart = millis();
      // get raw data
      ESP_LOGI(TAG, "Taking snapshot...");
      // use raw bmp image
      hw_camera_raw_snapshot(bmp_buf, &width, &height);
      elapsed_time = millis() - Tstart;
      ESP_LOGI(TAG, "Snapshot taken (%d) width: %d, height: %d", elapsed_time, width, height);
     // print_memory();
      // prepare feature
     
      Tstart = millis();
      ei::signal_t signal;      
      // generate feature
       ei_prepare_feature(bmp_buf, &signal);
      elapsed_time = millis() - Tstart;
      ESP_LOGI(TAG, "Feature taken (%d)", elapsed_time);
     // print_memory();
      // run classifier
   
      Tstart = millis();
      ei_impulse_result_t result = { 0 };
      bool debug_nn = false;
      // run classifier
      run_classifier(&signal, &result, debug_nn);
      elapsed_time = millis() - Tstart;
      ESP_LOGI(TAG, "Classification done (%d)", elapsed_time);
      //print_memory();
      // use result
      ei_use_result(result);
     


      String test;
      DynamicJsonDocument simplifiedtJsonDocument(1024); // Adjust the size as needed

        simplifiedtJsonDocument["count"] = order;
        simplifiedtJsonDocument["height"] = acc;

        // Serialize the simplified JSON document to a string
        serializeJson(simplifiedtJsonDocument, test);

        client.publish(topic, test.c_str());
        Serial.println("send data success");


        gi = !gi;
  }
  else{
    //senddata();
  }
  if (digitalRead(BTN_PIN) == 0) {
    if ((millis() - prev_millis > 500) && (press_state == false)) {
  
         press_state = true;
    } 
  } else {
    if (press_state) {
      press_state = false;
    }
    //senddata();
  }
  delay(100);
}

void initmap(){
  // Create 10 maps with "label_i" as keys and i as values
      std::map<std::string, float> current_map;
      std::string key = "108";
      current_map[key] = 108.00;
      array_of_maps.push_back(current_map);

      key = "109";
      current_map[key] = 109.00;
      array_of_maps.push_back(current_map);

      key = "110";
      current_map[key] = 110.00;
      array_of_maps.push_back(current_map);

      key = "111";
      current_map[key] = 111.00;
      array_of_maps.push_back(current_map);

      key = "112";
      current_map[key] = 112.00;
      array_of_maps.push_back(current_map);

      key = "113";
      current_map[key] = 113.00;
      array_of_maps.push_back(current_map);

      key = "114";
      current_map[key] = 114.00;
      array_of_maps.push_back(current_map);

      key = "115";
      current_map[key] = 115.00;
      array_of_maps.push_back(current_map);

      key = "116";
      current_map[key] = 116.00;
      array_of_maps.push_back(current_map);

      key = "117";
      current_map[key] = 117.00;
      array_of_maps.push_back(current_map);

      key = "118";
      current_map[key] = 118.00;
      array_of_maps.push_back(current_map);

      key = "119";
      current_map[key] = 119.00;
      array_of_maps.push_back(current_map);
}



void senddata(void){
camera_fb_t *fb = esp_camera_fb_get();

    if (fb) {
    Serial.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    }

}

void wificonnectSimple()
{
    WiFi.mode(WIFI_STA); //Optional
    WiFi.config(ip, gateway, subnet);
    WiFi.begin(WIFI_SSID);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}
void mqtest()
{
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callbackk);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {  //client.connect(client_id.c_str(), mqtt_username, mqtt_password)
            // Serial.println("Public EMQX MQTT broker connected");
        } else {
            // Serial.print("failed with state ");
            // Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe

  client.subscribe(topicsud);
}

void callbackk(char *topic, byte *payload, unsigned int length) {
  
   
  memcpy(cmd_buf, payload, length);
  cmd_buf[length] = '\0';
  deserializeJson(cmd_doc, cmd_buf);

  if (cmd_doc["cmd"] == "getdata") {
     gi = true;
    order = cmd_doc["counting"];
  }
}

// Print memory information
void print_memory() {
  ESP_LOGI(TAG, "Total heap: %u", ESP.getHeapSize());
  ESP_LOGI(TAG, "Free heap: %u", ESP.getFreeHeap());
  ESP_LOGI(TAG, "Total PSRAM: %u", ESP.getPsramSize());
  ESP_LOGI(TAG, "Free PSRAM: %d", ESP.getFreePsram());
}

// prepare feature
void ei_prepare_feature(uint8_t *img_buf, signal_t *signal) {
  signal->total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal->get_data = &ei_get_feature_callback;
  if ((EI_CAMERA_RAW_FRAME_BUFFER_ROWS != EI_CLASSIFIER_INPUT_WIDTH) || (EI_CAMERA_RAW_FRAME_BUFFER_COLS != EI_CLASSIFIER_INPUT_HEIGHT)) {
    ei::image::processing::crop_and_interpolate_rgb888(
      img_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      img_buf,
      EI_CLASSIFIER_INPUT_WIDTH,
      EI_CLASSIFIER_INPUT_HEIGHT);
  }
}

// get feature callback
int ei_get_feature_callback(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (bmp_buf[pixel_ix] << 16) + (bmp_buf[pixel_ix + 1] << 8) + bmp_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix+=3;
    pixels_left--;
  }
  return 0;
}

// use result from classifier
void ei_use_result(ei_impulse_result_t result) {

  float max = -1.0;
  std::string maxlabel;

  ESP_LOGI(TAG, "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
    result.timing.dsp, result.timing.classification, result.timing.anomaly);
  bool bb_found = result.bounding_boxes[0].value > 0;
  size_t i =  result.bounding_boxes_count;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {

    auto bb = result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }
    ESP_LOGI(TAG, "%s (%f) [ x: %u, y: %u, width: %u, height: %u ]", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    //Serial.println("%s (%f) ", bb.label, bb.value);
    jsonDocument["label"] = bb.label;
    jsonDocument["value"] = bb.value;
    jsonDocument["x"] = bb.x;
    jsonDocument["y"] = bb.y;
    jsonDocument["width"] = bb.width;
    jsonDocument["height"] = bb.height;

    if(ix == 0)
    {
      max = bb.value;
      maxlabel = std::string(bb.label);
    }  
    if(max < bb.value)
    {
      max == bb.value;
      maxlabel = std::string(bb.label);
    }

    // Serialize the JSON document to a string
    String jsonString;
    serializeJson(jsonDocument, jsonString);

    // Publish the JSON string to the MQTT topic
    client.publish(topiss, jsonString.c_str());
    //Serial.println("%s (%f) [ x: %u, y: %u, width: %u, height: %u ]", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

  }

// String to search for
    //std::string searchString = "da";

    // Iterate through the vector
    for (const auto& myMap : array_of_maps) {
        // Search for the string in the current map
        auto it = myMap.find(maxlabel);

        // Check if the string is found
        if (it != myMap.end()) {
            // Print the associated float value
            acc = it->second;
            break;  // Break out of the loop once found, assuming there is only one occurrence
        }
    }

  //acc = array_of_maps[maxlabel];
  if (!bb_found) {
    //ESP_LOGI(TAG, "No objects found");
    Serial.println("not valid");
     String testnon;
      DynamicJsonDocument simplifiedintJsonDocument(1024); // Adjust the size as needed

        simplifiedintJsonDocument["count"] = order;
        simplifiedintJsonDocument["height"] = "invalid";

        // Serialize the simplified JSON document to a string
        serializeJson(simplifiedintJsonDocument, testnon);
         gi = false;
        client.publish(topic, testnon.c_str());

  }
}