/**
 * @file config.h
 * @brief Hardware pinout configuration and tuning parameters for the 5-IR Line Follower Robot.
 * 
 * Hardware Architecture:
 * - Microcontroller: Arduino Uno (ATmega328P)
 * - Motor Driver: L298N Dual H-Bridge Module
 * - Sensors: 5-Channel Infrared (IR) Reflectance Sensor Array (15 mm spacing)
 * - Actuators: 2x 6V DC Geared Motors (10:1 Micro Metal Gearmotors, 1000 RPM rated)
 * - Power Source: 7.4V 2S Li-ion Battery Pack
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// IR Sensor Analog Input Pins (A0 - A4)
// Array Layout: [S0 (Far Left) ... S2 (Center) ... S4 (Far Right)]
// ==========================================
#define PIN_IR_FAR_LEFT     A0
#define PIN_IR_MID_LEFT     A1
#define PIN_IR_CENTER       A2
#define PIN_IR_MID_RIGHT    A3
#define PIN_IR_FAR_RIGHT    A4

// Number of optical sensors
#define NUM_SENSORS         5

// Sensor Analog Threshold: Readings below this value indicate dark line (0), above indicate white background (1)
// Inverted logic: 0 = Black Line Detected, 1 = White Surface
#define SENSOR_THRESHOLD    500

// ==========================================
// L298N Motor Driver Digital Pins
// EasyEDA Schematic: D2 -> ENA, D3 -> IN1, D4 -> IN2, D5 -> IN3, D6 -> IN4, D7 -> ENB
// ==========================================
#define PIN_MOTOR_ENA       2   // Left Motor PWM Enable
#define PIN_MOTOR_IN1       3   // Left Motor Direction 1
#define PIN_MOTOR_IN2       4   // Left Motor Direction 2
#define PIN_MOTOR_IN3       5   // Right Motor Direction 1
#define PIN_MOTOR_IN4       6   // Right Motor Direction 2
#define PIN_MOTOR_ENB       7   // Right Motor PWM Enable

// ==========================================
// Motor PWM Speed Parameters (0 - 255)
// ==========================================
#define BASE_SPEED          180 // Nominal forward driving speed
#define TURN_SPEED_FAST     180 // Pivot speed for outer wheel during turns
#define TURN_SPEED_SLOW     70  // Reduced inner wheel speed (alpha * v, alpha ≈ 0.38)
#define SHARP_TURN_SPEED    160 // Speed for differential contra-rotation turns

// Direction polarities
#define DIR_FORWARD         1
#define DIR_BACKWARD       -1
#define DIR_STOP            0

#endif // CONFIG_H
