#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "m_usb.h"
#define F_CPU 16000000UL
#define MAX_PWM_FREQ 200000UL
#define MIN_PWM_FREQ 1UL
#include <util/delay.h>

#define BUFFER_SIZE 32
char inputBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
uint8_t pwm1Started = 0;
uint8_t pwm3Started = 0;

void InitPWM_OC1A() {
	// PWM Output (D9, PB5 = OC1A)
	DDRB |= (1 << PB5);
	
	// Motor Control Pins
	DDRD |= (1 << PD5);  // TX LED, Motor Enable
	PORTD &= ~(1 << PD5);
	DDRD |= (1 << PD4);  // D4, Motor Direction
	PORTD |= (1 << PD4);
	
	// Timer1 Configuration for Fast PWM, Mode 14 (ICR1 as TOP)
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12); // Set mode first
	ICR1 = (F_CPU / 50000UL) - 1;  // Default 50 kHz
	OCR1A = ICR1 / 2; // 50% duty cycle
	TCNT1 = 0; // Reset counter
	TCCR1B |= (1 << CS10); // Start timer with no prescaler
}

void UpdatePWM_OC1A(uint32_t freq) {
	if (freq < MIN_PWM_FREQ || freq > MAX_PWM_FREQ) return;
	
	// Disable timer during update to prevent glitches
	uint8_t oldTCCR1B = TCCR1B;
	TCCR1B = (1 << WGM13) | (1 << WGM12); // Stop timer
	
	uint32_t top = (F_CPU / freq) - 1;
	if (top > 65535) top = 65535;
	if (top < 1) top = 1;
	
	ICR1 = (uint16_t)top;
	OCR1A = ICR1 / 2; // Maintain 50% duty cycle
	TCNT1 = 0; // Reset counter
	
	// Restore timer operation
	TCCR1B = oldTCCR1B;
}

void InitPWM_OC3A() {
	// PWM Output (D5, PC6 = OC3A)
	DDRC |= (1 << PC6);
	
	// Motor Control Pins
	DDRD |= (1 << PD7);  // D6, Motor Enable
	PORTD &= ~(1 << PD7);
	DDRD |= (1 << PD6);  // D12, Motor Direction
	PORTD &= ~(1 << PD6);
	
	// Timer3 Configuration for Fast PWM, Mode 14 (ICR3 as TOP)
	TCCR3A = (1 << COM3A1) | (1 << WGM31);
	TCCR3B = (1 << WGM33) | (1 << WGM32); // Set mode first
	ICR3 = (F_CPU / 50000UL) - 1; // Default 50 kHz
	OCR3A = ICR3 / 2; // 50% duty cycle
	TCNT3 = 0; // Reset counter
	TCCR3B |= (1 << CS30); // Start timer with no prescaler
}

void UpdatePWM_OC3A(uint32_t freq) {
	if (freq < MIN_PWM_FREQ || freq > MAX_PWM_FREQ) return;
	
	// Disable timer during update to prevent glitches
	uint8_t oldTCCR3B = TCCR3B;
	TCCR3B = (1 << WGM33) | (1 << WGM32); // Stop timer
	
	uint32_t top = (F_CPU / freq) - 1;
	if (top > 65535) top = 65535;
	if (top < 1) top = 1;
	
	ICR3 = (uint16_t)top;
	OCR3A = ICR3 / 2; // Maintain 50% duty cycle
	TCNT3 = 0; // Reset counter
	
	// Restore timer operation
	TCCR3B = oldTCCR3B;
}

void StopPWM_OC1A() {
	TCCR1A = 0;
	TCCR1B = 0;
	PORTB &= ~(1 << PB5);
	PORTD |= (1 << PD5); // Disable motor
	pwm1Started = 0;
}

void StopPWM_OC3A() {
	TCCR3A = 0;
	TCCR3B = 0;
	PORTC &= ~(1 << PC6);
	PORTD |= (1 << PD7); // Disable motor
	pwm3Started = 0;
}

uint32_t getFirstTwoDigits(uint32_t num) {
	char buffer[12]; // Enough for 32-bit numbers
	snprintf(buffer, sizeof(buffer), "%lu", num);
	buffer[2] = '\0'; // Truncate after 2nd digit
	return atol(buffer);
}

// Function to parse frequency string (like "30.10") to Hz (30100)
uint32_t parseFrequency(const char* str) {
	char* dot = strchr(str, '.');
	if (dot == NULL) {
		// No decimal point, treat as integer Hz
		return atol(str);
	}
	
	// Split into integer and fractional parts
	char intPart[16] = {0};
	char fracPart[16] = {0};
	
	strncpy(intPart, str, dot - str);
	strncpy(fracPart, dot + 1, sizeof(fracPart) - 1);
	
	uint32_t integer = atol(intPart);
	uint32_t fraction = atol(fracPart);
	
	uint32_t frac_2Digits = getFirstTwoDigits(fraction);
	
	// Calculate final frequency in Hz
	return (integer * 1000) + frac_2Digits*10;
}

void initialize(void) {
	m_usb_init();
	sei();
}

int main(void) {
	initialize();

	while (1) {
		if (m_usb_isconnected()) {
			if (m_usb_rx_available()) {
				char received_char = m_usb_rx_char();

				if (received_char == '\r' || received_char == '\n') {
					inputBuffer[bufferIndex] = '\0';

					if (strncmp(inputBuffer, "set ", 4) == 0) {
						char *token = strtok(&inputBuffer[4], " ");
						if (token != NULL) {
							uint32_t pwm1 = parseFrequency(token);
							token = strtok(NULL, " ");
							if (token != NULL) {
								uint32_t pwm3 = parseFrequency(token);

								if (pwm1 > 0) {
									if (!pwm1Started) {
										InitPWM_OC1A();
										pwm1Started = 1;
									}
									UpdatePWM_OC1A(pwm1);
									} else if (pwm1Started) {
									StopPWM_OC1A();
								}

								if (pwm3 > 0) {
									if (!pwm3Started) {
										InitPWM_OC3A();
										pwm3Started = 1;
									}
									UpdatePWM_OC3A(pwm3);
									} else if (pwm3Started) {
									StopPWM_OC3A();
								}

								// Format the output to show decimal point
								m_usb_tx_string("PWM updated to ");
								m_usb_tx_uint(pwm1 / 100);
								m_usb_tx_string(".");
								if ((pwm1 % 100) < 10) m_usb_tx_string("0");
								m_usb_tx_uint(pwm1 % 100);
								m_usb_tx_string("kHz and ");
								m_usb_tx_uint(pwm3 / 100);
								m_usb_tx_string(".");
								if ((pwm3 % 100) < 10) m_usb_tx_string("0");
								m_usb_tx_uint(pwm3 % 100);
								m_usb_tx_string("kHz\r\n");
								} else {
								m_usb_tx_string("Missing PWM3 value\r\n");
							}
							} else {
							m_usb_tx_string("Invalid set command\r\n");
						}
						} else if (strcmp(inputBuffer, "off") == 0) {
						StopPWM_OC1A();
						StopPWM_OC3A();
						m_usb_tx_string("Both PWM stopped\r\n");
						} else {
						m_usb_tx_string("Unknown command\r\n");
					}

					bufferIndex = 0;
					} else if (bufferIndex < BUFFER_SIZE - 1) {
					inputBuffer[bufferIndex++] = received_char;
					} else {
					bufferIndex = 0;
				}
			}
		}
		_delay_ms(10);
	}
}