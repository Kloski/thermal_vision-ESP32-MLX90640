#include <ESP8266WebServer.h>
#include <MLX90641_API.h>
#include <MLX90641_I2C_Driver.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>

#define ESP8266_DRD_USE_RTC false
#define ESP_DRD_USE_LITTLEFS true
#define DOUBLERESETDETECTOR_DEBUG false

#include <ESP_DoubleResetDetector.h>

// global settings
const char *sensor = "MLX90640";

const int rows = 16;
const int cols = 12;
const int total_pixels = rows * cols;
float frame[rows][cols];

const int humanThreshold = 5;

// camera settings
const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8                  //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[total_pixels];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;

// ESP server settgins
ESP8266WebServer server(80);
DoubleResetDetector *drd;

String output;

float getPixel(int x, int y)
{
    if (x < rows && x >= 0 && y < cols && y >= 0)
    {
        return frame[x][y];
    }
    return (float)0.0;
}

void getRaw()
{
    Serial.println("getRaw called - Starting MLX90641Frame computation");
    for (byte x = 0; x < 2; x++)
    {
        int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);

        float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
        float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);

        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    }
    Serial.println("Starting MLX90641Frame computation finished");

    String new_output;
    String data;

    StaticJsonDocument<1024> doc;

    bool person_detected = false;
    float personThreshold = 26;
    int col = 0;
    int row = 0;
    float pixel_temperature;
    float min = 0;
    float max = 0;
    float avg = 0;
    unsigned char min_index = 0;
    unsigned char max_index = 0;

    Serial.println("Starting frame construction");
    for (int i = 0; i < total_pixels; i++)
    {
        float cameraIndexVal = MLX90641To[i];
        frame[row][col] = cameraIndexVal;

        // Serial.print(cameraIndexVal);
        // Serial.print(" , ");

        if (row + 1 == rows)
        {
            row = 0;
            col++;
            // Serial.println("");
        }
        else
        {
            row++;
        }
    }

    Serial.println("Frame construction finished. Continuing with payload and person detection");

    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            int i = r + c * rows;
            pixel_temperature = getPixel(r, c);

            if (i == 0 || pixel_temperature > max)
            {
                max = pixel_temperature;
                max_index = i;
            }
            if (i == 0 || pixel_temperature < min)
            {
                min = pixel_temperature;
                min_index = i;
            }

            avg += pixel_temperature;

            data.concat(String(pixel_temperature, 2));

            if (i < total_pixels - 1)
            {
                data.concat(",");
            }

            // person detection
            if (pixel_temperature > personThreshold)
            {
                int pixelsExceededPersonThreshold = 1;

                if (getPixel(r + 1, c - 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r + 1, c) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r + 1, c + 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r, c + 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r, c - 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r - 1, c - 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r - 1, c) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }
                if (getPixel(r - 1, c + 1) > personThreshold - 2)
                {
                    pixelsExceededPersonThreshold++;
                }

                if (pixelsExceededPersonThreshold > humanThreshold)
                {
                    person_detected = true;
                }
            }
        }
    }
    Serial.println("Payload construction and payload detection finished");

    avg = avg / total_pixels;

    doc["sensor"] = sensor;
    doc["rows"] = rows;
    doc["cols"] = cols;
    doc["data"] = data.c_str();
    doc["temp"] = avg;
    doc["min"] = min;
    doc["max"] = max;
    doc["avg"] = avg;
    doc["min_index"] = min_index;
    doc["max_index"] = max_index;
    doc["overflow"] = false;
    doc["movingAverageEnabled"] = false;
    doc["interruptPinEnabled"] = false;
    doc["10fps"] = false;
    doc["person_detected"] = person_detected;

    Serial.println("output serializing");
    serializeJson(doc, new_output);

    output = new_output;
    Serial.println("get raw finished. New output computed");
}

void sendRaw()
{
    Serial.println("started collcting data and sending");
    getRaw();
    server.send(200, "application/json", output.c_str());
    Serial.println("data sent");
}

void restart()
{
    Serial.println("restarting ESP");
    server.send(200, "text/plain", "OK, let's do it!");
    delay(1000); // wait for sending response
    ESP.restart();
}

void notFound()
{
    Serial.println("notFound");
    server.send(404, "text/plain", "Not found");
}

//Returns true if the MLX90641 is detected on the I2C bus
boolean isConnected()
{
    Wire.beginTransmission((uint8_t)MLX90641_address);

    if (Wire.endTransmission() != 0)
    {
        return (false); //Sensor did not ACK
    }
    return (true);
}

void setup()
{
    Serial.begin(9600);

    Wire.begin();
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz

    if (isConnected() == false)
    {
        Serial.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1)
            ;
    }

    //Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status; //MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//

    if (status != 0)
    {
        Serial.println("Failed to load system parameters");
        while (1)
            ;
    }

    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    if (status != 0)
    {
        Serial.println("Parameter extraction failed");
        while (1)
            ;
    }

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x03); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz

    WiFi.mode(WIFI_STA);

    WiFiManager wm;

    drd = new DoubleResetDetector(10, 2);

    if (drd->detectDoubleReset())
    {
        Serial.println("Double reset detected");
        wm.resetSettings();
    }

    bool res2;
    res2 = wm.autoConnect("thermalvision");

    if (!res2)
    {
        Serial.println("Failed to connect");
        ESP.restart();
    }
    else
    {
        Serial.println("Connected");

        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }

        server.on("/raw", sendRaw);
        server.on("/restart", restart);
        server.onNotFound(notFound);

        Serial.println("server begin");
        server.begin();
        Serial.println("HTTP Server started");
    }
}

// https://forum.arduino.cc/t/sketch-to-convert-milliseconds-to-hours-minutes-and-seconds-hh-mm-ss/636386
// https://forum.arduino.cc/t/using-millis-for-timing-a-beginners-guide/483573
// https://www.arduino.cc/reference/en/language/functions/time/millis/
int getUptimeHours()
{
    long currentMillis = millis();
    long seconds = currentMillis / 1000;
    int minutes = seconds / 60;
    int hours = minutes / 60;
    //long days = hours / 24;

    // currentMillis %= 1000;
    // seconds %= 60;
    // minutes %= 60;
    // hours %= 24;

    // Serial.print("Uptime seconds: ");
    // Serial.println(seconds);
    // Serial.print("Uptime hours: ");
    // Serial.println(hours);

     return hours;
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        delay(100);
        server.handleClient();
    }
    if (getUptimeHours() > 6)
    {
        restart();
    }
    drd->loop();
}