#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <WiFi.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Firebase_ESP_Client.h>
#include <WebServer.h>
#include "index.h"  //Web page header file

WebServer server(80);


//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// size of noise sample
#define SAMPLES 1024
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;
#define OCTAVES 9

// Insert your network credentials
#define WIFI_SSID "zaidi&gang"
#define WIFI_PASSWORD "seecs1609"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCd4CZ1Xm27nDN1RH-QdCkJ2f5Gz_8lEQY"

#define USER_EMAIL "usamaqudoos@gmail.com"
#define USER_PASSWORD "123456789"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://esd-project-demo-default-rtdb.asia-southeast1.firebasedatabase.app/" 
//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String statusPath = "/status";
String timePath = "/timestamp";


// Parent Node (to be updated in every loop)
String parentPath;

FirebaseJson json;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 3000;

static const uint16_t timer_divider = 8;          // Divide 80 MHz by this
static const uint64_t timer_max_count = 1000000;  // Timer counts to this 
static hw_timer_t *timer = NULL;
static TaskHandle_t processing_task = NULL;

// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[OCTAVES];
// A-weighting curve from 31.5 Hz ... 8000 Hz
static const float aweighting[] = {-39.4, -26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1};
static unsigned int bell = 0;
static unsigned int fireAlarm = 0;
static unsigned long ts = millis();
static unsigned long last = micros();
static unsigned int sum = 0;
static unsigned int mn = 9999;
static unsigned int mx = 0;
static unsigned int cnt = 0;
static unsigned long lastTrigger[2] = {0, 0};

char* text = "Alarm off";


void IRAM_ATTR onTimer()
{
  BaseType_t task_woken = pdFALSE;
  size_t num_bytes_read;

   static int32_t samples[BLOCK_SIZE];
    // Read multiple samples at once and calculate the sound pressure
   // size_t num_bytes_read;
    esp_err_t err = i2s_read(I2S_PORT,
                             (char *)samples,
                             BLOCK_SIZE, // the doc says bytes, but its elements.
                             &num_bytes_read,
                             portMAX_DELAY); // no timeout
   // int samples_read = num_bytes_read / 8;
    // integer to float
    integerToFloat(samples, real, imag, SAMPLES);

    vTaskNotifyGiveFromISR(processing_task, &task_woken);

      if (task_woken) {
    portYIELD_FROM_ISR();
  }

}

//===============================================================
// Setup
//===============================================================
void setup(void){
  Serial.begin(115200);
  initWiFi();

  Serial.println();
  Serial.println("Booting Sketch...");


  Serial.println("Configuring I2S...");
    esp_err_t err;
  // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = 22627,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // for old esp-idf versions use RIGHT
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = 8,                       // number of buffers
        .dma_buf_len = BLOCK_SIZE,                // samples per buffer
        .use_apll = true};
    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // BCKL
        .ws_io_num = 15,    // LRCL
        .data_out_num = -1, // not used (only for speakers)
        .data_in_num = 32   // DOUTESP32-INMP441 wiring
    };
    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK)
    {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true)
            ;
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK)
    {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true)
            ;
    }
    Serial.println("I2S driver installed.");

    xTaskCreatePinnedToCore(vPeriodicTask,"PTask",1000,NULL,2,NULL,1);
    xTaskCreatePinnedToCore(vTaskFFT, "TaskFFT", 1000*32, NULL, 1, &processing_task, 1);
    xTaskCreatePinnedToCore(vTaskFirebase, "Firebase", 1000*32, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vPeriodicTask,"PTask2",1000,NULL,2,NULL,0);
    timer = timerBegin(0, timer_divider, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, timer_max_count, true);
    timerAlarmEnable(timer);

}

void vTaskFirebase(void *pvParameter){
  for(;;){
  // Send new readings to database
    if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
      sendDataPrevMillis = millis();

      //Get current timestamp
      timestamp = getTime();
      Serial.print ("time: ");
      Serial.println (timestamp);

      parentPath= databasePath + "/" + String(timestamp);
      
      json.set(statusPath.c_str(), String(text));
      json.set(timePath, String(timestamp));
      Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
      server.on("/readADC", handleADC);//To get update of ADC Value only
      
      
    }
    vTaskDelay(10000);
  }
}
void vPeriodicTask(void *pvParameters)
{
  for(;;)
  {
    int a = 0;
    a = a + 1; 
    vTaskDelay(10);
  }
  
}

void vTaskFFT(void *pvParameters) //Deffered Task
{
  for(;;){

    // Wait for notification from ISR (similar to binary semaphore)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // apply flat top window, optimal for energy calculations
    fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
    fft.Compute(FFT_FORWARD);
    // calculate energy in each bin
    calculateEnergy(real, imag, SAMPLES);
    // sum up energy in bin for each octave
    sumEnergy(real, energy, 1, OCTAVES);
    // calculate loudness per octave + A weighted loudness
    float loudness = calculateLoudness(energy, aweighting, OCTAVES, 1.0);
    unsigned int peak = (int)floor(fft.MajorPeak());
    //Serial.println(peak);
    
    // detecting 1kHz and 1.5kHz
    if (detectFrequency(&bell, 15, peak, 45, 68, true))
    {
        Serial.println("Detected bell");
        sendAlarm(0, "home/alarm/doorbell", 2000);
        text = "Bell Detected";   
    }
   
    //detecting frequencies around 3kHz
    else if (detectFrequency(&fireAlarm, 15, peak, 135, 160, true))
    {
        Serial.println("Detected fire alarm");
        sendAlarm(1, "home/alarm/fire", 10000);
        text = "FireAlarm Detected";
    }
    else {
      text = "Alarm Off";
    }
    calculateMetrics(loudness);
    vTaskDelay(500);
           }
}

void initWiFi() {

  Serial.println(WiFi.localIP());
  Serial.println();
  Wire.begin();
  Serial.println(F("MASTER NODE Test begin"));
  //WiFi.mode(WIFI_STA); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());  
  server.on("/", handleRoot);      //This is display page
  
  server.begin();                  //Start server
  Serial.println("HTTP server started");

  timeClient.begin();

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
}
void handleRoot() {
 String s = MAIN_page; //Read HTML contents
 server.send(200, "text/html", s); //Send web page
}
 
void handleADC() {
 String adcValue = String(text);
 
 server.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}
// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}
static void integerToFloat(int32_t *integer, float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = (integer[i] >> 16) / 10.0;
        vImag[i] = 0.0;
    }
}
// calculates energy from Re and Im parts and places it back in the Re part (Im part is zeroed)
static void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = sq(vReal[i]) + sq(vImag[i]);
        vImag[i] = 0.0;
    }
}
// sums up energy in bins per octave
static void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves)
{
    // skip the first bin
    int bin = bin_size;
    for (int octave = 0; octave < num_octaves; octave++)
    {
        float sum = 0.0;
        for (int i = 0; i < bin_size; i++)
        {
            sum += real[bin++];
        }
        energies[octave] = sum;
        bin_size *= 2;
    }
}
static float decibel(float v)
{
    return 10.0 * log(v) / log(10);
}
// converts energy to logaritmic, returns A-weighted sum
static float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale)
{
    float sum = 0.0;
    for (int i = 0; i < num_octaves; i++)
    {
        float energy = scale * energies[i];
        sum += energy * pow(10, weights[i] / 10.0);
        energies[i] = decibel(energy);
    }
    return decibel(sum);
}


unsigned int countSetBits(unsigned int n)
{
    unsigned int count = 0;
    while (n)
    {
        count += n & 1;
        n >>= 1;
    }
    return count;
}
//detecting 2 frequencies. Set wide to true to match the previous and next bin as well
bool detectFrequency(unsigned int *mem, unsigned int minMatch, double peak, unsigned int bin1, unsigned int bin2, bool wide)
{
    *mem = *mem << 1;
    if (peak == bin1 || peak == bin2 || (wide && (peak == bin1 + 1 || peak == bin1 - 1 || peak == bin2 + 1 || peak == bin2 - 1)))
    {
        *mem |= 1;
    }
    if (countSetBits(*mem) >= minMatch)
    {
        return true;
    }
    return false;
}
void sendAlarm(unsigned int index, char *topic, unsigned int timeout)
{
    // do not publish if last trigger was earlier than timeout ms
    if (abs(millis() - lastTrigger[index]) < timeout)
    {
        return;
    }
    lastTrigger[index] = millis();
    //publish to mqtt
    //publish(topic, "1");
}

void calculateMetrics(int val) {
  cnt++;
  sum += val;
  if (val > mx)
  {
      mx = val;
  }
  if (val < mn)
  {
      mn = val;
  }
}
void loop(void)
{
    server.handleClient();
    delay(1);
}

