/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This is the calibration sketch. Use it to determine the calibration_factor that the main example uses. It also
 outputs the zero_factor useful for projects that have a permanent mass on the scale in between power cycles.
 
 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Use this calibration_factor on the example sketch
 
 This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).
 
 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE

 Pico C SDK port
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "HX711.h"

#define LOADCELL_DOUT_PIN  18
#define LOADCELL_SCK_PIN  19


hx711_t scale;

float calibration_factor = 440500; //-7050 worked for my 440lb max scale setup

void setup() {
  printf("HX711 calibration sketch\n");
  printf("Remove all weight from scale\n");
  printf("After readings begin, place known weight on scale\n");
  printf("Press + or a to increase calibration factor\n");
  printf("Press - or z to decrease calibration factor\n");

  hx711_init(&scale, LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 128);
  hx711_set_scale(&scale, 1.0f);
  hx711_tare(&scale, 10);	//Reset the scale to 0

  long zero_factor = hx711_read_average(&scale, 10); //Get a baseline reading
  printf("Zero factor: %ld\n", zero_factor); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
}

int main()
{
    stdio_init_all();

    // Give time for USB serial connection to open
    sleep_ms(2000);

    setup();

    while (true) {
        hx711_set_scale(&scale, calibration_factor); //Adjust to this calibration factor

        printf("Reading: %.4f kg calibration_factor: %.0f\n", hx711_get_units(&scale, 1), calibration_factor);

        int temp = getchar_timeout_us(0);
        if(temp != PICO_ERROR_TIMEOUT)
        {
            if(temp == '+' || temp == 'a')
            calibration_factor += 100;
            else if(temp == '-' || temp == 'z')
            calibration_factor -= 100;
        }
        
        sleep_ms(100); // small delay to not flood the serial too quickly
    }
}