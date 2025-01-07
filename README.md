// https://github.com/Rio-The-Yash/Eng.-Project-3/blob/phase2/
// rm -rf runme && gcc pwm_dual_motor.c -o runme -l ncurses -l wiringPi
 
#include <stdio.h>
#include <signal.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <unistd.h>
#include <ncurses.h>

#define SIGALRM_INTERVAL 1 // 1 millisecond interval for SIGALRM

// Pin definitions for motor control
const int MotorDirFL = 0;  // GPIO pin position 11, controls the left motor in Forward direction
const int MotorDirBL = 1;  // GPIO pin position 12, controls the left motor in Backward direction
const int MotorPWM_L = 2;  // GPIO pin position 13, used to enable PWM for the left motor

const int MotorDirFR = 21;  // GPIO pin position 29, controls the right motor in Forward direction
const int MotorDirBR = 22;  // GPIO pin position 31, controls the right motor in Backward direction
const int MotorPWM_R = 23;  // GPIO pin position 33, used to enable PWM for the right motor

// Pin definitions for ultrasonic sensor
const int trigger_pin = 15;     // GPIO pin position 14
const int echo_pin = 16;        // GPIO pin position 15

// Global variables
const int PWM_Period = 100; // PWM period in milliseconds (100 ms cycle)
int duty_cycle = 0;         // Current duty cycle percentage (0 to 100%)
int running = 1;            // Program state flag (1 to keep running, 0 to stop)
int PWMCounter = 0;         // Counter to track the current position in the PWM period
#define SOUND_SPEED 0.0343  // Speed of sound in cm/µs
#define TIMEOUT 100000      // 100 ms timeout for ECHO signal
#define US_THRESHOLD 50     // Ultrasonic sensor threshold distance in cm
#define SAMPLING_RATE 250000   // 250 ms interval (in microseconds)
unsigned long t1, t2, t3;   // Timing variables for ultrasonic sensor

// SIGALRM handler to implement PWM control
void alarm_handler(int sig_num)
{
    PWMCounter++;  // Increment the counter

    // Reset the counter once a full PWM period (0 to 100) is completed
    if (PWMCounter >= PWM_Period) {
        PWMCounter = 0;
    }

    // Set the motor PWM pin HIGH within the duty cycle period, LOW otherwise
    if (PWMCounter < duty_cycle) {
        digitalWrite(MotorPWM_L, HIGH);  // Inside duty cycle, enable motor (set HIGH)
        digitalWrite(MotorPWM_R, HIGH);  // Inside duty cycle, enable motor (set HIGH)
    } else {
        digitalWrite(MotorPWM_L, LOW);   // Outside duty cycle, disable motor (set LOW)
        digitalWrite(MotorPWM_R, LOW);   // Outside duty cycle, disable motor (set LOW)
    }
}

// Function to initialize GPIO pins without turning off the motors
void init_state() {
    digitalWrite(MotorDirFL, LOW); // Motor forward direction off
    digitalWrite(MotorDirBL, LOW); // Motor backward direction off
    digitalWrite(MotorDirFR, LOW); // Motor forward direction off
    digitalWrite(MotorDirBR, LOW); // Motor backward direction off
    digitalWrite(MotorPWM_L, HIGH); // Enable PWM for left motor (initially HIGH)
    digitalWrite(MotorPWM_R, HIGH); // Enable PWM for right motor (initially HIGH)
}

// Function to stop motors by turning off only the direction pins
void stop_motor() {
    digitalWrite(MotorDirFL, LOW); // Turn off forward direction for left motor
    digitalWrite(MotorDirBL, LOW); // Turn off backward direction for left motor
    digitalWrite(MotorDirFR, LOW); // Turn off forward direction for right motor
    digitalWrite(MotorDirBR, LOW); // Turn off backward direction for right motor
    printw("Motor stopped\n");
    refresh();
}

// Setup function to initialize motor pins and PWM settings
void setup_PWM()
{
    pinMode(MotorDirFL, OUTPUT);  // Left motor forward pin
    pinMode(MotorDirBL, OUTPUT);  // Left motor backward pin
    pinMode(MotorPWM_L, OUTPUT);  // Left motor PWM control pin
    pinMode(MotorDirFR, OUTPUT);  // Right motor forward pin
    pinMode(MotorDirBR, OUTPUT);  // Right motor backward pin
    pinMode(MotorPWM_R, OUTPUT);  // Right motor PWM control pin

    init_state();  // Set initial motor state
}

// Function to initialize ultrasonic sensor pins
void init_ultrasonic() {
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    digitalWrite(trigger_pin, LOW);
}

// Function to calculate distance
double getDistance() {

    digitalWrite(trigger_pin, LOW);
    // Wait for 2 microseconds using micros()
    unsigned long startWait = micros();
    while ((micros() - startWait) < 2);

    digitalWrite(trigger_pin, HIGH);
    unsigned long triggerStart = micros();
    while ((micros() - triggerStart) < 10); // Busy-wait for 10 microseconds
    digitalWrite(trigger_pin, LOW);
    t1 = micros(); // Record time when TRIGGER is set LOW 

    // Wait for ECHO to go HIGH (within 100 ms)
    startWait = micros();
    while (digitalRead(echo_pin) == LOW) {
        if (micros() - startWait > TIMEOUT) { // Timeout after 100 ms
            printw("Timeout waiting for ECHO to go HIGH\n");
            return -1.0;
        }
    }
    t2 = micros(); // Record time when ECHO goes HIGH

    // Wait for ECHO to go LOW (within 100 ms)
    startWait = micros();
    while (digitalRead(echo_pin) == HIGH) {
        if (micros() - startWait > TIMEOUT) { // Timeout after 100 ms
            printw("Timeout waiting for echo to go LOW\n");
            return -1.0;
        }
    }
    t3 = micros(); // Record time when ECHO_pinecho_pin goes LOW

    // Calculate the distance using t2 and t3
    unsigned long travelTime = t3 - t2; // Time in microseconds
    double distance = (travelTime * SOUND_SPEED) / 2.0;

    // Return None if out-of-range reading (less than 2 cm or greater than 400 cm)
    if (distance < 2 || distance > 400) {
        return -1.0; // Indicate an invalid reading
    }

    return distance;
}

// Function to change the duty cycle based on user input
void change_duty_cycle(char input) {
    switch(input) {
        case 'm':
            duty_cycle = 0.90 * PWM_Period;
            printw("Duty Cycle set to 90%%\n");
            break;
        case 'n':
            duty_cycle = 0.65 * PWM_Period;
            printw("Duty Cycle set to 65%%\n");
            break;
        case 'q':
            duty_cycle = 0;
            stop_motor();  // Stop the motor instead of calling init_state
            printw("Motor turned off\n");
            break;
    }
    refresh();
}

void move_Forward() {
    init_state();
    digitalWrite(MotorDirFR, HIGH); // Forward on
    digitalWrite(MotorDirFL, HIGH); // Forward on
    printw("Going in Forward Direction\n"); 
    refresh();
}

void move_Backward() {
    init_state();
    digitalWrite(MotorDirBL, HIGH); // Backward on
    digitalWrite(MotorDirBR, HIGH); // Backward on
    printw("Going in Backward Direction\n");
    refresh();

    // Monitor ultrasonic sensor while moving backward
    while (running) {

        unsigned long wait = micros();
        double distance = getDistance();
        if (distance < 0) {
            printw("Ultrasonic sensor error or timeout\n");
        } else {
            printw("Distance: %.2f cm\n", distance);
        }
        refresh();
        if (distance > 0 && distance <= US_THRESHOLD) {
            stop_motor();
            printw("Obstacle detected! Stopping motor\n");
            refresh();
            break;
        }
        while ((micros() - wait) < SAMPLING_RATE); // Busy-wait for 250 ms
    }
}

void move_Left() {
    init_state();
    digitalWrite(MotorDirFR, HIGH); // Forward on
    digitalWrite(MotorDirBL, HIGH); // Backward on
    printw("Going in Left Direction\n");
    refresh();
}

void move_Right() {
    init_state();
    digitalWrite(MotorDirFL, HIGH); // Forward on
    digitalWrite(MotorDirBR, HIGH); // Backward on
    printw("Going in Right Direction\n");
    refresh();
}

// Setup function for wiringPi, PWM, and SIGALRM signal handling
void setup() {
    wiringPiSetup();
    setup_PWM();
    init_ultrasonic();

    // Initialize ncurses
    initscr();
    noecho();
    nodelay(stdscr, FALSE); // Enable blocking mode
    printw("Enter w (forward), s (backward), m (90%% duty cycle), n (75%% duty cycle), q (stop motor), x (terminate program)\n");
    refresh();
    
    signal(SIGALRM, alarm_handler);
    ualarm(500000 , SIGALRM_INTERVAL * 1000);
}

void terminate() {
    running = 0;
    stop_motor();  // Call stop_motor instead of manually turning off pins
    printw("Program Terminated Successfully\n");
    refresh();
    endwin();  // End ncurses mode
}

int main(int argc, char const *argv[])
{
    setup();

    while(running) {
        char input = getch();
        
        switch (input) {
        case 'x':
            terminate();
            break;
        case 'm':
        case 'n':
        case 'q':
            change_duty_cycle(input);
            break;
        case 'w':
            move_Forward();
            break;
        case 'a':
            move_Left();
            break;
        case 's':
            move_Backward();
            break;
        case 'd':
            move_Right();
            break;
        default:
            printw("Invalid Input\n");
            break;
        }
    }
    
    return 0;
}
