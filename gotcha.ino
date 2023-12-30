#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"

const char* ssid = "wifi_name";
const char* password = "wifi_password";

//FLASH RELATED INITIALIZATIONS
#define LAMP_PIN 4
int lampChannel = 7;           // a free PWM channel (some channels used by camera)
unsigned int Status;

const int pwmfreq = 50000;     // 50K pwm frequency
const int pwmresolution = 9;   // duty cycle bit range
const int pwmMax = pow(2,pwmresolution)-1;

void setup() {
  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//ESP32-CAM camera configuration
void configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5; config.pin_d1 = 18;
  config.pin_d2 = 19; config.pin_d3 = 21;
  config.pin_d4 = 36; config.pin_d5 = 39;
  config.pin_d6 = 34; config.pin_d7 = 35;
  config.pin_xclk = 0; config.pin_pclk = 22;
  config.pin_vsync = 25; config.pin_href = 23;
  config.pin_sscb_sda = 26; config.pin_sscb_scl = 27;
  config.pin_pwdn = 32; config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // initialization of image quality size and count
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init test to see if it is initialized correctly
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
}

void loop() {
  configInitCamera();
  delay(1000);

  // Simulate the photoshoot by turning on the lamp for 1 second
  setLamp(100);
  delay(1000);
  setLamp(0);

  // Tested out the camera capture
  run_face_detect();
  delay(1000);
}

bool run_face_detect() {
  bool faceRecognised = false;
  int64_t start_time = esp_timer_get_time();
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  Serial.println("Camera capture successfull");

  int64_t fb_get_time = esp_timer_get_time();
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);

  if (!res)
  {
    Serial.println("to rgb888 failed");
    dl_matrix3du_free(image_matrix);
  }

  esp_camera_fb_return(fb);
  box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

  if (net_boxes) {

    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
    {
      Serial.println("Face Detected");
      Status = 1;
    }
    else
    {
      Serial.println("Face Not Aligned");
    }
  }

  else
  {
    Serial.println("No Face Detected");
  }

  dl_matrix3du_free(image_matrix);
  return faceRecognised;
}

void setLamp(int newVal) {
    if (newVal != -1) {
        // Apply a logarithmic function to the scale.
        int brightness = round((pow(2,(1+(newVal*0.02)))-2)/6*pwmMax);
        ledcWrite(lampChannel, brightness);
        Serial.print("Lamp: ");
        Serial.print(newVal);
        Serial.print("%, pwm = ");
        Serial.println(brightness);
    }
}

