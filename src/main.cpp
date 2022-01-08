// https://community.platformio.org/t/adafruit-busio-adafruit-spidevice-h17-fatal-error-spi-h-no-such-file-or-directory/14864
#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_MLX90640.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>

#define ESP8266_DRD_USE_RTC false
#define ESP_DRD_USE_LITTLEFS true
#define DOUBLERESETDETECTOR_DEBUG false

#include <ESP_DoubleResetDetector.h>

Adafruit_MLX90640 mlx;

ESP8266WebServer server(80);
DoubleResetDetector *drd;

const unsigned long getFrameInterval = 1000;
unsigned long previousFrameTime = 0;

const float humanThreshold = 3.5;

const int rows = 16;
const int cols = 12;
const int total_pixels = rows * cols;
float pixels[total_pixels];
float frame[rows][cols];

String output;

const char *sensor = "MLX90640";

float getPixel(int x, int y)
{
    if (x <= rows && x > 0 && y <= cols && y > 0)
    {
        return frame[x][y];
    }
    return (float)0.0;
}

void getRaw()
{
    String new_output;
    String data;

    StaticJsonDocument<4096> doc;

    bool person_detected = false;
    float personThreshold = 0;
    int col = 0;
    int row = 0;
    float pixel_temperature;
    float min = 0;
    float max = 0;
    float avg = 0;

    for (int i = 0; i < total_pixels; i++)
    {
        pixel_temperature = pixels[i];

        if (i == 0 || pixel_temperature > max)
        {
            max = pixel_temperature;
        }
        if (i == 0 || pixel_temperature < min)
        {
            min = pixel_temperature;
        }

        avg += pixel_temperature;

        data.concat(String(pixel_temperature, 1));

        if (i < total_pixels - 1)
        {
            data.concat(",");
        }
    }

    avg = avg / total_pixels;
    personThreshold = humanThreshold + avg;

    for (int i = 0; i < total_pixels; i++)
    {
        frame[row][col] = pixels[i];
        if (col == cols)
        {
            col = 0;
            row++;
        }
        else
        {
            col++;
        }
    }

    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (getPixel(r, c) > personThreshold)
            {
                int blobSize = 1;

                if (getPixel(r + 1, c - 1) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r + 1, c) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r + 1, c + 1) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r, c + 1) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r, c - 1) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r - 1, c - 1) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r - 1, c) > personThreshold - 2)
                {
                    blobSize++;
                }
                if (getPixel(r - 1, c + 1) > personThreshold - 2)
                {
                    blobSize++;
                }

                if (blobSize > 4)
                {
                    person_detected = true;
                }
            }
        }
    }

    doc["sensor"] = sensor;
    doc["rows"] = rows;
    doc["cols"] = cols;
    doc["data"] = data.c_str();
    doc["min"] = min;
    doc["max"] = max;
    doc["avg"] = avg;
    doc["person_detected"] = person_detected;
    // doc["person_detected"] = max - min > 4;

    serializeJson(doc, new_output);
    output = new_output;
}

void sendRaw()
{
    getRaw();
    server.send(200, "application/json", output.c_str());
}

void notFound()
{
    server.send(404, "text/plain", "Not found");
}

void setup()
{
    while (!Serial)
        delay(10);
    Serial.begin(9600);

    Serial.println("Adafruit MLX90640");
    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
    {
        Serial.println("MLX90640 not found!");
        while (1)
            delay(10);
    }
    Serial.println("Found Adafruit MLX90640");

    Serial.print("Serial number: ");
    Serial.print(mlx.serialNumber[0], HEX);
    Serial.print(mlx.serialNumber[1], HEX);
    Serial.println(mlx.serialNumber[2], HEX);

    mlx.setMode(MLX90640_CHESS);
    Serial.print("Current mode: ");
    if (mlx.getMode() == MLX90640_CHESS)
    {
        Serial.println("Chess");
    }
    else
    {
        Serial.println("Interleave");
    }

    mlx.setResolution(MLX90640_ADC_18BIT);
    Serial.print("Current resolution: ");
    mlx90640_resolution_t res = mlx.getResolution();
    switch (res)
    {
    case MLX90640_ADC_16BIT:
        Serial.println("16 bit");
        break;
    case MLX90640_ADC_17BIT:
        Serial.println("17 bit");
        break;
    case MLX90640_ADC_18BIT:
        Serial.println("18 bit");
        break;
    case MLX90640_ADC_19BIT:
        Serial.println("19 bit");
        break;
    }

    mlx.setRefreshRate(MLX90640_4_HZ);
    Serial.print("Current frame rate: ");
    mlx90640_refreshrate_t rate = mlx.getRefreshRate();
    switch (rate)
    {
    case MLX90640_0_5_HZ:
        Serial.println("0.5 Hz");
        break;
    case MLX90640_1_HZ:
        Serial.println("1 Hz");
        break;
    case MLX90640_2_HZ:
        Serial.println("2 Hz");
        break;
    case MLX90640_4_HZ:
        Serial.println("4 Hz");
        break;
    case MLX90640_8_HZ:
        Serial.println("8 Hz");
        break;
    case MLX90640_16_HZ:
        Serial.println("16 Hz");
        break;
    case MLX90640_32_HZ:
        Serial.println("32 Hz");
        break;
    case MLX90640_64_HZ:
        Serial.println("64 Hz");
        break;
    }

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
        Wire.begin();
        // Library assumes "Wire" for I2C but you can pass something else with begin() if you like

        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        server.on("/raw", sendRaw);

        server.onNotFound(notFound);

        server.begin();
        Serial.println("HTTP Server started");
    }
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        delay(100);
        server.handleClient();
    }
    drd->loop();
}
