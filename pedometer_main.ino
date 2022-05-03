/*
  edit outside arduino ide:
  arduino ide > File > Preference > Use external editor
*/

#include <math.h>
#include <Wire.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define SDA                 12                      // Data pin
#define SCL                 14                      // Clock pin
#define MPU_ADD             104                     // MPU9250 address on I2C 
#define ACC_MEAS_REG        59                      // Address of acceleation register
#define CAL_NO              1000                    // Number of callibration samples
#define SENSOR_FREQ         0.01                    // 0.01Hz       
#define NOISE_MEAN          0.07138525875587912     // Mean of sensor noise
#define WIN_WIDTH           50                      // Width after which we re-compute the mean
#define SSID                "your_ssid"
#define PASS                "your_password"
#define DEBUG_MODE          0

uint8_t accMeas[] = {0,0,0,0,0,0};
float   acc_x_off = 0.0;                            
float   acc_y_off = 0.0;
float   acc_z_off = 0.0;
float   acc_x     = 0.0;
float   acc_y     = 0.0;
float   acc_z     = 0.0;
float   acc       = 0.0;
int   steps_taken =   0;
float max_thresh  = 0.0;
float min_thresh  = 0.0;
float mean_thresh = 0.0;
Ticker read_sensor;

// ESP8266WebServer server;
WiFiServer server(80);
/////////////////////////////////////////////////////////////////////////////////////
void I2CWriteRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t Value)
{
  // I2CDevice  - I2C device address.
  // RegAddress - Device Register address.
  // Value      - Value to write in register.
  
  Wire.beginTransmission(I2CDevice);
  
  // Inform device from which register we want to read / write.
  Wire.write(RegAdress);
  
  // Send value
  Wire.write(Value);
  
  Wire.endTransmission();
}

void I2CReadRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t NBytes, uint8_t *Value)
{
  Wire.beginTransmission(I2CDevice);
  
  // Inform device from which register we want to read / write.
  Wire.write(RegAdress);
  
  // End transmission:
  Wire.endTransmission();
  
  // Napravi sporočimo, da želimo prebrati določeno število 8-bitnih registrov:
  // Tell device, we want to NBytes of 8-bit registers
  Wire.requestFrom(I2CDevice, NBytes);
  
  for (int q = 0; q < NBytes; q++) 
  {
    // Read 8-bit register
    *Value = (uint8_t)Wire.read();
    Value++;
    
    //uint32_t vrednost = Wire.read();
  }
}
///////////////////////////////////////I2C////////////////////////////////////////////
void MPU9250_init()
{
  // Reset MPU9250 sensor, by writting one to register PWR_MGMT_1 (107) 
  I2CWriteRegister(MPU_ADD,107,128); // 128 = 1000 0000
  delay(500);
  
  // Read sensor ID, by reading register WHO_AM_I (117)  
  uint8_t ID;
  I2CReadRegister(MPU_ADD,117,1,&ID);
  Serial.println("ID:");
  Serial.println(ID, HEX);
  
  // Gyroscope Conf => Register GYRO_CONFIG (27) 
  // 4 in 3 bit dolocata obseg 
  I2CWriteRegister(MPU_ADD,27,0); // 
  delay(100);
  
  // Accelerator Conf => Register ACCEL_CONFIG (28)
  // 4 in 3 bit dolocata obseg 
  // Opciono => Register ACCEL_CONFIG_2 (29)
  I2CWriteRegister(MPU_ADD,28,0); // 
  delay(100);
}

void calibrateAcc()
{
  Serial.println("Calibration...");
  
  int32_t tmp;
  for(int i=0; i< CAL_NO; i++)
  {
     I2CReadRegister(MPU_ADD, ACC_MEAS_REG, 6, accMeas);

     tmp = (((int8_t)accMeas[0] << 8) + (uint8_t)accMeas[1]);
     acc_x_off += tmp * 1.0/16384.0;

     tmp = (((int8_t)accMeas[2] << 8) + (uint8_t)accMeas[3]);
     acc_y_off += tmp * 1.0/16384.0;
   
     tmp = (((int8_t)accMeas[4] << 8) + (uint8_t)accMeas[5]);
     acc_z_off += tmp * 1.0/16384.0;
  }

  acc_x_off /= CAL_NO;
  acc_y_off /= CAL_NO;
  acc_z_off /= CAL_NO;


#if DEBUG_MODE
  Serial.print("acc_x_off = ");
  Serial.print(acc_x_off);
  Serial.print(" ");

  Serial.print("acc_y_off = ");
  Serial.print(acc_y_off);
  Serial.print(" ");

  Serial.print("acc_z_off = ");
  Serial.print(acc_z_off);
  Serial.println();
#endif
}
///////////////////////////////////////FILTERS//////////////////////////////////////
float IIR(float acc_0)
{
  // Cut off frequency of 3Hz.
  static const float A[] = {0.85435899};
  static const float B[] = {0.07282051, 0.07282051};
  static float acc_1 = 0.0f;
  static float acc_f = 0.0f;

  acc_f = A[0] * acc_f + B[0] * acc_0  + B[1] * acc_1;
  acc_1 = acc_0;

  return acc_f;
}

float BUTTER_WORTH(float acc_0)
{
  // Cut off frequency of 3Hz.
  static const float A[] = {1.73550016, -0.76660814};
  static const float B[] = {0.007777, 0.01555399, 0.007777};

  static float acc_1  = 0.0f;
  static float acc_2  = 0.0f;
  static float acc_f0 = 0.0f;
  static float acc_f1 = 0.0f;
  static float acc_f2 = 0.0f;

  acc_f0 = A[0] * acc_f1 + A[1] * acc_f2 
            + B[0] * acc_0 + B[1] * acc_1 + B[2] * acc_2;

  acc_2  = acc_1;
  acc_1  = acc_0;
  acc_f2 = acc_f1;
  acc_f1 = acc_f0;

  return acc_f0;
}
////////////////////////////////////////////////////////////////////////////////
void mean_window(float acc_f)
{
  static int iter = 0;
  static float _max = -100000.0;
  static float _min = +100000.0;  
  if (acc_f > _max)
    _max = acc_f;

  else if (acc_f < _min)
    _min = acc_f;

  if (iter == WIN_WIDTH)
  {

    max_thresh = _max;
    min_thresh = _min;
    mean_thresh = (max_thresh + min_thresh ) / 2.0f;
    
    // Reset iterator
    iter = 0;

    // Reset min and max
    _max = -100000.0;
    _min = +100000.0;
  }

#if DEBUG_MODE
  Serial.print("a_max = ");
  Serial.print(max_thresh);
  Serial.print(" ");
  Serial.print("a_min = ");
  Serial.print(min_thresh);
  Serial.print(" ");
  Serial.print("a_mean = ");
  Serial.print(mean_thresh);
  Serial.print(" ");
#endif
  
  iter += 1;
}

void detect_step(float acc_0)
{
  static float acc_1 = 0.0;

  //if (mean_thresh > NOISE_MEAN)
  if ((max_thresh - min_thresh) > NOISE_MEAN)
  {
    if (acc_0 < mean_thresh && acc_1 > mean_thresh)
    {
      steps_taken += 1;
    }
  }

  acc_1 = acc_0;
}
///////////////////////////////////////////////////////////////////////////////
void read_acc()
{
  // Read accelometer
  int32_t tmp;
  I2CReadRegister(MPU_ADD, ACC_MEAS_REG, 6, accMeas);
  tmp = (((int8_t)accMeas[0] << 8) + (uint8_t)accMeas[1]);
  acc_x = tmp * 1.0/16384.0;
  tmp = (((int8_t)accMeas[2] << 8) + (uint8_t)accMeas[3]);
  acc_y = tmp * 1.0/16384.0;
  tmp = (((int8_t)accMeas[4] << 8) + (uint8_t)accMeas[5]);
  acc_z = tmp * 1.0/16384.0;
  
  // Subtract the offset
  acc_x -= acc_x_off;
  acc_y -= acc_y_off;
  acc_z -= acc_z_off;

  acc = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  //Serial.print("a = ");
  //Serial.print(acc);
  //Serial.print(" ");

  // Filter the raw acceleration signal
  /*
  float acc_f = IIR(acc);
  Serial.print("a_f = ");
  Serial.print(acc_f);
  Serial.print(" "); 
  */
  
  float acc_f = BUTTER_WORTH(acc);

#if DEBUG_MODE
  Serial.print("a_f = ");
  Serial.print(acc_f);
  Serial.print(" ");

  Serial.print("steps_taken = ");
  Serial.print(steps_taken);
  Serial.print(" "); 
#endif

  mean_window(acc_f);
  detect_step(acc_f);

  Serial.println();
}
/////////////////////////////////////////////////////////////////////////////////////
String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
String html_1 = R"=====(<!DOCTYPE html>
<html>
 <head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset='utf-8'>
  <style>
          html {
           font-family: Arial;
           display: inline-block;
           margin: 0px auto;
           text-align: center;
          }
          h2 { font-size: 3.0rem; }
          p { font-size: 3.0rem; }
          .units { font-size: 1.2rem; }
          .dht-labels{
            font-size: 1.5rem;
            vertical-align:middle;
            padding-bottom: 15px;
          }
  </style>
 
  <script> 
    function updateCount() 
    {  
       ajaxLoad('getCount'); 
    }
 
 
    var ajaxRequest = null;
    if (window.XMLHttpRequest)  { ajaxRequest =new XMLHttpRequest(); }
    else                        { ajaxRequest =new ActiveXObject("Microsoft.XMLHTTP"); }
 
    function ajaxLoad(ajaxURL)
    {
      if(!ajaxRequest){ alert('AJAX is not supported.'); return; }
 
      ajaxRequest.open('GET',ajaxURL,true);
      ajaxRequest.onreadystatechange = function()
      {
        if(ajaxRequest.readyState == 4 && ajaxRequest.status==200)
        {
          var ajaxResult = ajaxRequest.responseText;
          document.getElementById('count_P').innerHTML = ajaxResult;
        }
      }
      ajaxRequest.send();
    }
 
    setInterval(updateCount, 500);
 
  </script>
    <title>Step Tracker</title>
    <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
 </head>
 
 <body>
    <h2 style='color:#0c796a'>Step Tracker</h2>   
    <p>
      <i class='fas fa-shoe-prints' style='font-size:45px; color:#17ab97'></i>
    </p>
     <div id='count_DIV'> 
       <p id='count_P'>
       Count: 0
       </p>
     </div>
   </div> 
 </body>
</html>
)====="; 

void init_server()
{
  //WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  // Wait until we connect to the network
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
   }

  Serial.println("");
  Serial.println("IP ADDR: ");
  Serial.println(WiFi.localIP());

  server.begin();
}
/////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  delay(1000);
  init_server();
  delay(2000);
  MPU9250_init();
  
  calibrateAcc();
  read_sensor.attach(SENSOR_FREQ, read_acc); 
}

String request = "";
void loop() 
{
  //server.handleClient();

  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) 
    return;
    
  // Read the first line of the request
  request = client.readStringUntil('\r');

  if (request.indexOf("getCount") > 0 )
  { 
    client.print( header );
    client.print( "Count = " ); 
    client.print( steps_taken ); 
  }

  else
  {
    client.flush();
    client.print( header );
    client.print( html_1 ); 
  }

  delay(5);
}
