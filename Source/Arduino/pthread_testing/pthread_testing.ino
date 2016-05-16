/* Copyright (c) 2011, Jan Clement
 * licenced under the GPL
 *
 * Author: Jan Clement <jan.clement@audiokits.de>
 * 
 * Example code to demonstrate the use of protothreads 
 * in an arduino sketch. It toggles an LED  using two 
 * independent protothreads. One pt toggles every 
 * 1000ms, the other one every 900ms. The result is an 
 * erratic blinking pattern.
 */

#include <pt.h>   // include protothread library

#define LEDPIN1 6
#define LEDPIN2 8
#define LEDPIN3 10
#define LEDPIN4 12

bool check = true;

static struct pt pt1, pt2, pt3, pt4; // each protothread needs one of these

void setup() {
  pinMode(LEDPIN1, OUTPUT); // LED init
  pinMode(LEDPIN2, OUTPUT); // LED init
  pinMode(LEDPIN3, OUTPUT); // LED init
  pinMode(LEDPIN4, OUTPUT); // LED init
  PT_INIT(&pt1);  // initialise the
  PT_INIT(&pt2);  // protothread variables
  PT_INIT(&pt3);
  PT_INIT(&pt4);
}

int shift = 700;

void toggleLED(int led_pin) {
  boolean ledstate = digitalRead(led_pin); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(led_pin, ledstate); // write inversed state back
}

/* This function toggles the LED after 'interval' ms passed */
static int protothread1(struct pt *pt, int interval) {
  static unsigned long timestamp = shift*0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    /* each time the function is called the second boolean
    *  argument "millis() - timestamp > interval" is re-evaluated
    *  and if false the function exits after that. */
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > interval );
    timestamp = millis(); // take a new timestamp
    toggleLED(LEDPIN1);
  }
  PT_END(pt);
}
/* exactly the same as the protothread1 function */
static int protothread2(struct pt *pt, int interval) {
  static unsigned long timestamp = shift*1;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > interval );
    timestamp = millis();
    toggleLED(LEDPIN2);
  }
  PT_END(pt);
}
static int protothread3(struct pt *pt, int interval) {
  static unsigned long timestamp = shift*2;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > interval );
    timestamp = millis();
    toggleLED(LEDPIN3);
  }
  PT_END(pt);
}
static int protothread4(struct pt *pt, int interval) {
  static unsigned long timestamp = shift*3;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > interval );
    timestamp = millis();
    toggleLED(LEDPIN4);
  }
  PT_END(pt);
}

void loop() {
  int delay = 400;
  protothread1(&pt1, delay); // schedule the protothreads
  protothread2(&pt2, delay); // by calling them infinitely
  protothread3(&pt3, delay);
  protothread4(&pt4, delay);
}