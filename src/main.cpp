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

// camera settings
const byte MLX90641_address = 0x33; // Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8                  // Default shift for MLX90641 in open air
// camera resolution
const int rows = 16;
const int cols = 12;
float frame[rows][cols];
const int total_pixels = rows * cols;
// camera frame
float MLX90641To[total_pixels];
uint16_t eeMLX90641[832];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;

// person detection values - can be configured via request params
// http://192.168.1.123/update?personThresholdLow=30&personThresholdHigh=40&humanThreshold=2&personTempDecrease=2
int humanThreshold = 3;
float tempKoef = 1.6;
float minHumanTemp = 25.5;
int minNeighboursCount = 2;
int delayOutputComputation = 8;
long mainLoopCounter = delayOutputComputation;

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

int getNeighboursCount(int r, int c, float threashold)
{
    int pixelsExceededPersonThreshold = 0;

    if (getPixel(r + 1, c - 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r + 1, c) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r + 1, c + 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r, c + 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r, c - 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r - 1, c - 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r - 1, c) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    if (getPixel(r - 1, c + 1) > threashold)
    {
        pixelsExceededPersonThreshold++;
    }
    return pixelsExceededPersonThreshold;
}

void refreshCameraTempsFrame()
{
    Serial.println("getRaw called - Starting MLX90641 Frame computation");
    for (byte x = 0; x < 2; x++)
    {
        int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
        Serial.print("Frame data status: ");
        Serial.println(status);

        float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
        Serial.print("vdd: ");
        Serial.println(vdd);

        float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);

        float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    }
    Serial.println("Starting MLX90641 Frame computation finished");

    // ####################################################################################################################

    Serial.println("Starting temp frame construction");
    int col = 0;
    int row = 0;
    for (int i = 0; i < total_pixels; i++)
    {
        frame[row][col] = MLX90641To[i];

        if (row + 1 == rows)
        {
            row = 0;
            col++;
        }
        else
        {
            row++;
        }
    }
    Serial.println("Temp frame construction finished");
}

void getRaw(int humanThreshold, float tempKoef)
{
    Serial.println("Starting payload construction");

    // std::map<int, int> tempCountMap = {};
    float avgTemp = 0;
    float min = 0;
    float max = 0;
    unsigned char min_index = 0;
    unsigned char max_index = 0;
    String data;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            int i = r + c * rows;
            float pixel_temperature = getPixel(r, c);
            avgTemp += pixel_temperature / total_pixels;

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

            data.concat(String(pixel_temperature, 2));

            if (i < total_pixels - 1)
            {
                data.concat(",");
            }

            // https://en.cppreference.com/w/cpp/container/map/find
            // int intTemp = static_cast<int>(pixel_temperature);
            // if (auto search = tempCountMap.find(intTemp); search != tempCountMap.end())
            // {
            //     // found
            //     tempCountMap[intTemp]++;
            // }
            // else
            // {
            //     // not found
            //     tempCountMap[intTemp] = 1;
            // }
        }
    }

    Serial.println("Payload construction finished");

    // ####################################################################################################################

    Serial.println("Person detection started");

    const float tempThreshold = avgTemp + ((avgTemp - min) * tempKoef);
    int tempAboveThresholdCount = 0;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++) // same cycle as above
        {
            float pixel_temperature = getPixel(r, c);
            if (tempThreshold < pixel_temperature && getNeighboursCount(r, c, tempThreshold) >= minNeighboursCount)
            {
                tempAboveThresholdCount++;
            }
        }
    }

    Serial.println("Person detection finished");

    // ####################################################################################################################

    Serial.println("Start building response payload");

    String new_output;
    StaticJsonDocument<1024> doc;

    doc["sensor"] = "MLX90641";
    doc["rows"] = rows;
    doc["cols"] = cols;
    doc["data"] = data.c_str();
    doc["temp"] = avgTemp;
    doc["avg"] = avgTemp;
    doc["min"] = min;
    doc["max"] = max;
    doc["min_index"] = min_index;
    doc["max_index"] = max_index;
    doc["overflow"] = false;
    doc["movingAverageEnabled"] = false;
    doc["person_detected"] = tempAboveThresholdCount >= humanThreshold && max >= minHumanTemp;

    // std::string tempCountMapCsv = "";
    // for (auto it = tempCountMap.cbegin(); it != tempCountMap.cend(); it++)
    // {
    //     std::string convrt = std::to_string(it->second);
    //     tempCountMapCsv += (it->first) + ":" + (convrt) + ", ";
    // }
    // doc["tempCountMap"] = tempCountMapCsv.substr(0, tempCountMapCsv.size() - 2);

    Serial.println("output serializing");
    serializeJson(doc, new_output);

    output = new_output;
    Serial.println("get raw finished. New output computed");
}

void updateProperties()
{
    Serial.print("updateProperties called - argument parsing. URL: ");
    String uri = server.uri(); // Get paths // https://stackoverflow.com/questions/69142021/grab-full-url-from-esp8266webserver
    Serial.println(uri);

    // https://forum.arduino.cc/t/esp8266-webserver-handling-multiple-requests/607950/4
    // https://forum.arduino.cc/t/is-this-the-best-way-to-get-data-from-a-http-request/678197/12
    String argsString = "{";
    // argsString += server.args(); // Get number of parameters
    for (int i = 0; i < server.args(); i++)
    {
        if (i > 0)
        {
            argsString += ",";
        }
        argsString += "\n\"";
        String argName = server.argName(i);
        String argValue = server.arg(i);
        // argsString += "Arg " + (String)i + ": "; // Include the current iteration value
        argsString += argName + "\": "; // Get the name of the parameter
        argsString += argValue;         // Get the value of the parameter

        if (argName == "humanThreshold")
        {
            Serial.print("Changing humanThreshold (");
            Serial.print(humanThreshold);
            Serial.print(") to: ");
            humanThreshold = atoi(argValue.c_str());
            Serial.println(humanThreshold);
        }
        else if (argName == "tempKoef")
        {
            Serial.print("Changing tempKoef (");
            Serial.print(tempKoef);
            Serial.print(") to: ");
            tempKoef = atof(argValue.c_str());
            Serial.println(tempKoef);
        }
        else if (argName == "minHumanTemp")
        {
            Serial.print("Changing minHumanTemp (");
            Serial.print(minHumanTemp);
            Serial.print(") to: ");
            minHumanTemp = atof(argValue.c_str());
            Serial.println(minHumanTemp);
        }
        else if (argName == "minNeighboursCount")
        {
            Serial.print("Changing minNeighboursCount (");
            Serial.print(minNeighboursCount);
            Serial.print(") to: ");
            minNeighboursCount = atof(argValue.c_str());
            Serial.println(minNeighboursCount);
        }
        else if (argName == "delayOutputComputation")
        {
            Serial.print("Changing delayOutputComputation (");
            Serial.print(delayOutputComputation);
            Serial.print(") to: ");
            delayOutputComputation = atof(argValue.c_str());
            Serial.println(delayOutputComputation);
        }
    }
    argsString += "\n}";
    Serial.println(argsString);

    server.send(200, "application/json", argsString.c_str());
    Serial.println("updateProperties called - arguments parsed");
}

void sendRaw()
{
    Serial.println("sendRaw called");
    server.send(200, "application/json", output.c_str());
    Serial.println("sendRaw finished - data sent");
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

// Returns true if the MLX90641 is detected on the I2C bus
boolean isConnected()
{
    Wire.beginTransmission((uint8_t)MLX90641_address);

    if (Wire.endTransmission() != 0)
    {
        return (false); // Sensor did not ACK
    }
    return (true);
}

void setup()
{
    Serial.begin(9600);

    Wire.begin();
    Wire.setClock(400000); // Increase I2C clock speed to 400kHz

    if (isConnected() == false)
    {
        Serial.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1)
            ;
    }

    // Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    int errorno = status; // MLX90641_CheckEEPROMValid(eeMLX90641); //eeMLX90641[10] & 0x0040; //
    Serial.print("errorno: ");
    Serial.println(errorno);

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

    // MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x03); // Set rate to 4Hz
    // MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz

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
        server.on("/update", updateProperties);
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
    // long days = hours / 24;

    return hours;
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        delay(100);
        server.handleClient();
    }
    if (getUptimeHours() > 1)
    {
        restart();
    }
    if (mainLoopCounter > delayOutputComputation)
    {
        mainLoopCounter = 0;
        Serial.println("frame reconstruction started");
        refreshCameraTempsFrame();
        Serial.println("frame reconstruction finished -> building output started");
        getRaw(humanThreshold, tempKoef);
        Serial.println("building output finished");
    }

    drd->loop();
    mainLoopCounter++;
    // Serial.print("Loop counter: ");
    // Serial.println(mainLoopCounter);
}
