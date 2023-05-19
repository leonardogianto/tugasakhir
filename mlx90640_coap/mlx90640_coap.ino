#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <CoAPSimple.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

//#define CONFIG_FREERTOS_TASK_STACK_SIZE 4096

static float mlx90640To[768];
paramsMLX90640 mlx90640;

WiFiClient wifiClient;
CoAPSimple coap;

const char* coapServer = "192.168.132.111";
const int coapPort = 5683;

const char* ssid = "Leo";
const char* password = "135792468";

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0){
    Serial.println("Parameter extraction failed");
    Serial.print(" status = ");
    Serial.println(status);
  }

  //Once params are extracted, we can release eeMLX90640 array

  MLX90640_I2CWrite(0x33, 0x800D, 6401);
  MLX90640_SetRefreshRate(MLX90640_address, 0x04);
  
  //ESP32 Connect to WiFi
  WiFi.mode(WIFI_STA); //Connect to wifi
  WiFi.begin(ssid, password);

  Serial.println("Connecting to ");
  Serial.print(ssid);

  //Wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){      
    Serial.print(".");
  }
    
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to ESP
  
  coap.begin();
}

void loop(){
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }
  float maxread = -273.0;
  for (int x = 0 ; x < 10 ; x++)
  {
    if (mlx90640To[x] > maxread){
      maxread = mlx90640To[x];
    }
  }
  if (maxread < 40)
  {
    char data[8];
    dtostrf(maxread, 1, 2, data);
    Serial.println(data);
    CoAPResponse response = coap.post(coapServer, coapPort, "/temperature", data);
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
