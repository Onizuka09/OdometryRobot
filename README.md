a line following robot using stm32F4 and implementing PID controller
## PINOUTs 
### LED
- Internal LED  PA5 
### PONT-H 
- IN1 PB3  (D3)	
- IN2 PB5  (D4)	
- IN3 PB4  (D5)
- IN4 PB1  (A3)

- ENA Motor Left: PC1  PWM_timaer15_ch1   
- ENB Motor Right:  PC2  PWM_timer15_ch2  

left:  

- EconderA OUTA : PA8 (D7) tim1_ch1 (encoder mode ) 
- EncoderA OUTB : PA9 (D8) tim1_ch2 (encoder mode )

right: 

- EconderB OUTA : PC7 (D9) TIM3_ch2 (encoder mode )
- EncoderB OUTB : PC6 (x) TIM3_ch1 (encoder mode )

# UART3: UART Communication with ESP32 
- PB8 : UART3_Tx 
- PB9 : UART3_Rx 

## Wiring  Encoders

- RED : motor+
- black: motor-
- green: GND
- blue: Vcc 
- Yellow: OUTA 
- White: OUTB 

const float wheelRadius = 6.7/2; // Radius of the wheel in meters (e.g., 5 cm)
// for full rotation the encoder does 13 puleses 
// Gear box  is 34 : 1 ratio 
const int pulsesPerRevolution = 34 * 13 ; // Number of pulses per revolution of the encoder

// Distance per pulse
const float distancePerPulse = (2 * 3.14159 * wheelRadius) / pulsesPerRevolution;

## PWM Frequencies for Motor control 
Recommended PWM Frequencies for DC Motors
Motor Type	Ideal Frequency Range	Why?
Small Brushed DC (e.g., hobby motors)	5kHz - 20kHz	Above audible range, good torque control
Large Brushed DC	500Hz - 5kHz	Lower switching losses in power drivers
Coreless Motors	20kHz - 50kHz	Avoids coil resonance frequencies



## STM32G070-NUCLEO_PINOUTS
![image](./Docs/stm32g070-NUCLEO_PINOUTS.png)