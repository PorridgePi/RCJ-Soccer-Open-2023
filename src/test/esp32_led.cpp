#include <Arduino.h>
#include <Led.h>

Led led;

void setup() {
    led.begin(18, 10);
    for (int i = 0; i < 10; i++) {
        led.color(i, 100,100,100,100);
        //led.color(i, 255*(0.5*(1+sin(0.01*millis()))), 10, 10, 10);
    }
    led.show();

}
void loop() {
    //led.begin(18, 10);
    /*for (int i = 0; i < 10; i++) {
        led.color(i, 10,10,10,10);
        //led.color(i, 255*(0.5*(1+sin(0.01*millis()))), 10, 10, 10);
    }
    led.show();*/
}

void loop() {
    // led.begin(18, 10);
    // for (int i = 0; i < 10; i++) {
    //     led.color(i, 255, 255, 255, 255);
    // }
    // led.show();
}
