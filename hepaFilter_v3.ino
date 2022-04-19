#include "Tlc5940.h"
#include <EEPROM.h>

#define DEBUG
#define ZERO_DETECT_PIN 2       // the number of the zeroDetct pin.
#define PIN_PWM 8               // Motor PWM Pin

#define PIN_SEG_A 4
#define PIN_SEG_B 12
#define PIN_SEG_C 6
#define PIN_SEG_D 7

#define PIN_BUZZ 19
#define PIN_LED 5				// For BreathEffect

#define CHK_BTN_AUTO 1
#define CHK_BTN_SLEEP 2
#define CHK_BTN_ON_OFF 4
#define CHK_BTN_SPEED 8
#define CHK_BTN_TIMER 16

#define LED_ON 4095
#define LED_DIM 1024
#define LED_OFF 0
#define LED_PM25_LOW 5      	// Blue
#define LED_PM25_MED 3      	// Yellow
#define LED_PM25_HIGH 0     	// Red
#define LED_PM01_LOW 6      	// Blue
#define LED_PM01_MED 4      	// Yellow
#define LED_PM01_HIGH 1     	// Red
#define LED_POWER 2         	// Red
#define LED_FAN 12          	// Blue
#define LED_AUTO_MODE 7     	// Blue
#define LED_SLEEP 9         	// Blue
#define LED_SPEED_LOW 11    	// Blue
#define LED_SPEED_MED 10    	// Blue
#define LED_SPEED_HIGH 8    	// Blue
#define LED_SPEED_LOW_DIM 13 	// For ledStatus
#define LED_SPEED_MED_DIM 14    // For ledStatus
#define LED_SPEED_HIGH_DIM 15   // For ledStatus

#define NUM_LED_POWER 4
#define NUM_LED_AUTO 128
#define NUM_LED_SPEED_HIGH 256
#define NUM_LED_SLEEP 512
#define NUM_LED_SPEED_MED 1024
#define NUM_LED_SPEED_LOW 2048
#define NUM_LED_FAN 4096
#define NUM_LED_SPEED_LOW_DIM 8192
#define NUM_LED_SPEED_MED_DIM 16384
#define NUM_LED_SPEED_HIGH_DIM 32768

#define BUZZ_MAX_TIME 5

#define TIM0_OVF 249     // Interrupt overflow count 1mSec

#define SPEED_MULTIPLIER 1
#define MAX_FAN_SPEED 6
#define MAX_HOURS 8
#define ONE_HOUR_COUNT 58600//3600000
#define DISP_OFF 10

#define PM01_LOW 12
#define PM01_MED 30
#define PM01_HIGH 35
#define PM25_LOW 50//12
#define PM25_MED 148//30
#define PM25_HIGH 170//35
#define PM01_LEVEL_GOOD 0
#define PM01_LEVEL_LOW 1
#define PM01_LEVEL_MED 2
#define PM01_LEVEL_HIGH 3
#define PM25_LEVEL_GOOD 0
#define PM25_LEVEL_LOW 1
#define PM25_LEVEL_MED 2
#define PM25_LEVEL_HIGH 3
volatile unsigned char airQualityPM01, airQualityPM25;
// Buzzer variables
volatile int buzzCount = 0;
volatile bool isBuzz = false;
// Buttons variables
volatile bool isAutoPressed = false;
volatile bool isSleepPressed = false;
volatile bool isPowerPressed = false;
volatile bool isSpeedPressed = false;
volatile bool isTimerPressed = false;
volatile bool isAutoReleased = false;
volatile bool isSleepReleased = false;
volatile bool isPowerReleased = false;
volatile bool isSpeedReleased = false;
volatile bool isTimerReleased = false;

volatile unsigned char hoursCount = 0;
volatile long hoursCounter = 0;
volatile unsigned char fanSpeed = 0;
volatile unsigned char pwmCount = 0;
volatile bool flagON = false;

#define STATE_POWER_OFF 0
#define STATE_POWER_ON 1
#define STATE_AUTO 2
#define STATE_SLEEP 3
volatile unsigned char sysState = STATE_POWER_OFF;
volatile unsigned char autoCounter = 0;
volatile unsigned int autoEnableCounter = 0;
volatile bool breathEffect = false;
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
volatile unsigned char countBreath = 0;
volatile bool isTimer = false;
volatile bool isSleep = false;
#define AUTO_MAX_COUNTER 4000
uint16_t ledStatus = 0;

// Function declarations
void Enable_Timer0(void);
void dispSegment(unsigned char);

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_01um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

uint16_t map_p01;
uint16_t map_p25;
struct pms5003data data;

//EEPROM
/** the current address in the EEPROM  **/
int addr = 0;

//Interrupt Service Routine for PCINT1(PC0-PC4), Panel Buttons
ISR(PCINT1_vect) {
    // ON/OFF
    if(((PINC & CHK_BTN_ON_OFF) == CHK_BTN_ON_OFF) && !isPowerPressed) {
        digitalWrite(PIN_BUZZ, LOW);        isBuzz = true;      isPowerPressed = true;
    } else if(((PINC & CHK_BTN_ON_OFF) == 0) && isPowerPressed) {
        isPowerReleased = true;         isPowerPressed = false;
    }
    if(sysState == STATE_POWER_ON || sysState == STATE_AUTO)
    {
        // Auto
        if(((PINC & CHK_BTN_AUTO) == CHK_BTN_AUTO) && !isAutoPressed && !isAutoReleased && !isSleep) {
            digitalWrite(PIN_BUZZ, LOW);    isBuzz = true;      isAutoPressed = true;
        } else if(((PINC & CHK_BTN_AUTO) == 0) && isAutoPressed) {
            isAutoReleased = true;          isAutoPressed = false;
        }
        // Sleep
        if(((PINC & CHK_BTN_SLEEP) == CHK_BTN_SLEEP) && !isSleepPressed && !isSleepReleased) {
            digitalWrite(PIN_BUZZ, LOW);    isBuzz = true;      isSleepPressed = true;
        } else if(((PINC & CHK_BTN_SLEEP) == 0) && isSleepPressed) {
            isSleepReleased = true;         isSleepPressed = false;
        }
        if(sysState == STATE_POWER_ON && !isSleep) {
            // Speed
            if(((PINC & CHK_BTN_SPEED) == CHK_BTN_SPEED) && !isSpeedPressed && !isSpeedReleased) {
                digitalWrite(PIN_BUZZ, LOW);    isBuzz = true;      isSpeedPressed = true;
            } else if(((PINC & CHK_BTN_SPEED) == 0) && isSpeedPressed) {
                isSpeedReleased = true;         isSpeedPressed = false;
            }
            // Timer
            if(((PINC & CHK_BTN_TIMER) == CHK_BTN_TIMER) && !isTimerPressed && !isTimerReleased) {
                digitalWrite(PIN_BUZZ, LOW);    isBuzz = true;      isTimerPressed = true;
            } else if(((PINC & CHK_BTN_TIMER) == 0) && isTimerPressed) {
                isTimerReleased = true;         isTimerPressed = false;
            }
        }
    }
}

// Timer0 ISR after every 1mSec
ISR(TIMER0_COMPA_vect) {
    if(flagON) {	// For Motor Control Only
        pwmCount--;
        if(pwmCount <= 0) {
            digitalWrite(PIN_PWM, LOW);     flagON = false;
        }
        else if(pwmCount == (fanSpeed*SPEED_MULTIPLIER)) {
            digitalWrite(PIN_PWM, HIGH);
        }
    }
    
    if(sysState == STATE_POWER_ON || sysState == STATE_AUTO) {
		if(isTimer) {
			hoursCounter--;
			if(hoursCounter < 1) {
				isPowerReleased = true;		isTimer = false;
			}
			if(!isSleep) {	
				dispSegment(ceil(float(hoursCounter)/float(ONE_HOUR_COUNT)));
			}
		}
		// --> Block modification needed
        autoEnableCounter++;
        if(autoEnableCounter > AUTO_MAX_COUNTER) {
            autoEnableCounter = 0;
            switch(autoCounter)
            {
                case 1:
                    sysState = STATE_AUTO;
                    autoCounter = 0;
                    break;
                case 2:
                    sysState = STATE_POWER_ON;
                    autoCounter = 0;
                    break;
                case 3:
                    breathEffect = true;
                    break;
                case 4:
                    breathEffect = false;
                    break;
                default:
                    autoCounter = 0;
            }
        }
    }
    
    if(isBuzz == true) {
        buzzCount++;
        if(buzzCount > BUZZ_MAX_TIME) {
            buzzCount = 0;
            digitalWrite(PIN_BUZZ, HIGH);
            isBuzz = false;
        }
    }
	if(breathEffect == true) {
		countBreath++;
		if(countBreath >= 30) {
			countBreath = 0;
			// set the brightness
			analogWrite(PIN_LED, brightness);

			// change the brightness for next time through the loop:
			brightness = brightness + fadeAmount;

			// reverse the direction of the fading at the ends of the fade:
			if (brightness <= 0 || brightness >= 255) {
				fadeAmount = -fadeAmount;
			}
		}
	}
}

void setup() {
    Serial.begin(9600);
    pinMode(PIN_LED, OUTPUT);      // initialize the LED pin
    pinMode(PIN_BUZZ, OUTPUT);      // initialize the Buzzer pin
    digitalWrite(PIN_BUZZ, HIGH);   // Turn OFF buzzer
    initSegment();
    dispSegment(DISP_OFF);
    pinMode(ZERO_DETECT_PIN, INPUT);  // initialize the zeroDetct pin as an input:
    pinMode(PIN_PWM, OUTPUT);  // initialize the PWM pin as an input:
    digitalWrite(PIN_PWM, LOW);
    cli(); // clear Global Interrupt
    attachInterrupt(digitalPinToInterrupt(ZERO_DETECT_PIN), zeroDetect_ISR, CHANGE);
    initButtons();
    Enable_Timer0();
    Tlc.init();
    Tlc.clear();
    Tlc.update();
    sei(); // Enable the Global Interrupts
	fanSpeed = EEPROM.read(addr); // Restore the speed
	breathEffect = false;
}

void loop() {
	
    if (readPMSdata(&Serial) && (sysState == STATE_POWER_ON || sysState == STATE_AUTO)) {
        map_p01 = data.pm10_standard;	//map(data.particles_01um, 0, 65535, 0, 1000);
        map_p25 = data.pm25_standard;	//map(data.particles_25um, 0, 65535, 0, 1000);

        if(map_p01 < PM01_LOW) {  // Clean Air PM03
            set_LED_PM01(PM01_LEVEL_GOOD);          airQualityPM01 = PM01_LEVEL_GOOD;
        }
        else if(map_p01 > PM01_LOW && map_p01 < PM01_MED) {  // Medium Air PM03
            set_LED_PM01(PM01_LEVEL_LOW);           airQualityPM01 = PM01_LEVEL_LOW;
        }
        else if(map_p01 > PM01_MED && map_p01 < PM01_HIGH) {  // Dirty Air PM03
            set_LED_PM01(PM01_LEVEL_MED);           airQualityPM01 = PM01_LEVEL_MED;
        }
        else if(map_p01 > PM01_HIGH) {  // Dirty Air PM03
            set_LED_PM01(PM01_LEVEL_HIGH);          airQualityPM01 = PM01_LEVEL_HIGH;
        }

        if(map_p25 < PM25_LOW) {  // Clean Air PM25
            set_LED_PM25(PM25_LEVEL_GOOD);          airQualityPM25 = PM25_LEVEL_GOOD;
        }
        else if(map_p25 > PM25_LOW && map_p25 < PM25_MED) {  // Medium Air PM25
            set_LED_PM25(PM25_LEVEL_LOW);           airQualityPM25 = PM25_LEVEL_LOW;
        }
        else if(map_p25 > PM25_MED && map_p25 < PM25_HIGH) {  // Dirty Air PM25
            set_LED_PM25(PM25_LEVEL_MED);           airQualityPM25 = PM25_LEVEL_MED;
        }
        else if(map_p25 > PM25_HIGH) {  // Dirty Air PM03
            set_LED_PM25(PM25_LEVEL_HIGH);          airQualityPM25 = PM25_LEVEL_HIGH;
        }

        if(sysState == STATE_AUTO) {
            if(airQualityPM25 == PM25_LEVEL_GOOD) {
                fanSpeed = 0;	EEPROM.write(addr, fanSpeed);	set_LED_Speed(fanSpeed);
            }
            else if(airQualityPM25 == PM25_LEVEL_LOW) {
                 fanSpeed = 2;	EEPROM.write(addr, fanSpeed);	set_LED_Speed(fanSpeed);
            }
            else if(airQualityPM25 == PM25_LEVEL_MED) {
                 fanSpeed = 4;	EEPROM.write(addr, fanSpeed);	set_LED_Speed(fanSpeed);       
            }
            else if(airQualityPM25 == PM25_LEVEL_HIGH) {
                 fanSpeed = 6;	EEPROM.write(addr, fanSpeed);	set_LED_Speed(fanSpeed);       
            }
        }
		Tlc.update();
    }
	switch(sysState)
    {
        case STATE_POWER_OFF:
        {
            if(isPowerReleased) {   // Turn ON Power
                set_LED_Power(true);
                set_LED_Speed(fanSpeed);
                Tlc.update();
                autoCounter = 0; // --> See later
                sysState = STATE_POWER_ON;
				isPowerReleased = false;
            }
			break;
		}
		case STATE_POWER_ON:
        {
            if(isPowerReleased) {   // Turn OFF Power
				hoursCount = 0;
                hoursCounter = 0;
                dispSegment(DISP_OFF);
                set_LED_Power(false);
                set_LED_PM01(0);
                set_LED_PM25(0);
                set_LED_Speed(0);
				set_LED_Auto(false);
				EEPROM.write(addr, fanSpeed);
                Tlc.update();
                autoCounter = 0;
                sysState = STATE_POWER_OFF;
				isPowerReleased = false;
            }
			if(isSpeedReleased) {
                fanSpeed++;
                if(fanSpeed > MAX_FAN_SPEED)
                    fanSpeed = 1;
				EEPROM.write(addr, fanSpeed);
                set_LED_Speed(fanSpeed);
                Tlc.update();
				isSpeedReleased = false;
            }
			if(isTimerReleased) {
				isTimer = false;
                hoursCount++;
                if(hoursCount > MAX_HOURS)
                    hoursCount = 0;
                dispSegment(hoursCount);
                hoursCounter = hoursCount * ONE_HOUR_COUNT;
				if(hoursCounter == 0)
					dispSegment(DISP_OFF);
				else
					isTimer = true;
				isTimerReleased = false;
            }
			if(isSleepReleased) {
                isSleepReleased = false;
                if(!isSleep) {
					dispSegment(DISP_OFF);
					Tlc.clear();
					Tlc.update();
					isSleep = true;
				}else if(isSleep) {
					isSleep = false;
					if(isTimer)
						dispSegment(hoursCount);
					LEDs_Restore();
				}
            }
			break;
		}
	}
}

void zeroDetect_ISR() {
    if(digitalRead(ZERO_DETECT_PIN) == 1 && fanSpeed != 0) {
        Enable_Timer0();
        pwmCount = 10;
        digitalWrite(PIN_PWM, LOW);
        if(fanSpeed == MAX_FAN_SPEED)
            digitalWrite(PIN_PWM, HIGH);
        flagON = true;
    }
}

boolean readPMSdata(Stream *s) {
    if (! s->available()) {
        return false;
    }
  
    // Read a byte at a time until we get to the special '0x42' start-byte
    if (s->peek() != 0x42) {
        s->read();
        return false;
    }
 
    // Now read all 32 bytes
    if (s->available() < 32) {
        return false;
    }
    
    uint8_t buffer[32];    
    uint16_t sum = 0;
    s->readBytes(buffer, 32);
    
    // get checksum ready
    for (uint8_t i=0; i<30; i++) {
        sum += buffer[i];
    }
 
  // The data comes in endian'd, this solves it so it works on all platforms
    uint16_t buffer_u16[15];
    for (uint8_t i=0; i<15; i++) {
        buffer_u16[i] = buffer[2 + i*2 + 1];
        buffer_u16[i] += (buffer[2 + i*2] << 8);
    }
 
    // put it into a nice struct :)
    memcpy((void *)&data, (void *)buffer_u16, 30);
    
    if (sum != data.checksum) {
        return false;
    }
    // success!
    return true;
} 

void initButtons(void) {
    DDRC   &= ~((1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4));       // Set as input (Using for interupt PCINT1)
    PORTC  |= ((1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3) | (1 << PORTC4));     // Enable pull-up resistor
    // Setting Pin Change ISR for Buttons
    PCMSK1 |= ((1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12));     // want pin PCINT8-12
    PCIFR  |= (1 << PCIF1);      // clear any outstanding interrupts
    PCICR  |= (1 << PCIE1);      // enable pin change interrupts for PCINT8-12

}

void Enable_Timer0(void) {
    TCCR0B &= ~((1 << CS00) | (1 << CS01) | (1 << CS02)) ;	// Turn OFF Timer
    // Setting Timer1 for PWM control
    TCCR0A |= (1 << WGM01);                      			// Set the Timer Mode to CTC
    TCCR0B |= (1 << CS00) | (1 << CS01) ;        			// set pre-scaler to 64 and start the timer
    OCR0A = TIM0_OVF;                                 		// Set 249 as value to count to
    TIMSK0 |= (1 << OCIE0A);                     			//Set the ISR COMPA vector
}

void initSegment(void) {
    // 7-Segment Pins Init
    pinMode(PIN_SEG_A, OUTPUT);  // initialize A
    pinMode(PIN_SEG_B, OUTPUT);  // initialize B
    pinMode(PIN_SEG_C, OUTPUT);  // initialize C
    pinMode(PIN_SEG_D, OUTPUT);  // initialize D
}

void dispSegment(unsigned char digit) {
	if(!isSleep) {
		switch(digit)
		{
			case 0:
				digitalWrite(PIN_SEG_A, LOW);   digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, LOW);
				break;
			case 1:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, LOW);
				break;
			case 2:
				digitalWrite(PIN_SEG_A, LOW);   digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, LOW);
				break;
			case 3:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, LOW);
				break;
			case 4:
				digitalWrite(PIN_SEG_A, LOW);   digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, LOW);
				break;
			case 5:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, LOW);
				break;
			case 6:
				digitalWrite(PIN_SEG_A, LOW);   digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, LOW);
				break;
			case 7:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, LOW);
				break;
			case 8:
				digitalWrite(PIN_SEG_A, LOW);   digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, HIGH);
				break;
			case 9:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, LOW);   digitalWrite(PIN_SEG_C, LOW);   digitalWrite(PIN_SEG_D, HIGH);
				break;
			case 10:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, HIGH);
				break;
			default:
				digitalWrite(PIN_SEG_A, HIGH);  digitalWrite(PIN_SEG_B, HIGH);  digitalWrite(PIN_SEG_C, HIGH);  digitalWrite(PIN_SEG_D, HIGH);
				break;
		}
	}
}

void set_LED_Power(bool b1) {
	if(!isSleep) {
		if(b1 == true)      Tlc.set(LED_POWER, LED_ON);
		else                Tlc.set(LED_POWER, LED_OFF);
	}
	if(b1 == true)      ledStatus |= (1 << LED_POWER);
	else				ledStatus &= ~(1 << LED_POWER);
	
}

void set_LED_Sleep(bool b1) {
	if(!isSleep) {
		if(b1 == true)      Tlc.set(LED_SLEEP, LED_ON);
		else                Tlc.set(LED_SLEEP, LED_OFF);
	}
	if(b1 == true)      ledStatus |= (1 << LED_SLEEP);
	else				ledStatus &= ~(1 << LED_SLEEP);
}

void set_LED_Fan(bool b1) {
	if(!isSleep) {
		if(b1 == true)      Tlc.set(LED_FAN, LED_ON);
		else                Tlc.set(LED_FAN, LED_OFF);
	}
	if(b1 == true)      ledStatus |= (1 << LED_FAN);
	else				ledStatus &= ~(1 << LED_FAN);
}

void set_LED_Auto(bool b1) {
	if(!isSleep) {
		if(b1 == true)      Tlc.set(LED_AUTO_MODE, LED_ON);
		else                Tlc.set(LED_AUTO_MODE, LED_OFF);
	}
	if(b1 == true)      ledStatus |= (1 << LED_AUTO_MODE);
	else				ledStatus &= ~(1 << LED_AUTO_MODE);
}

void set_LED_Speed(unsigned char uc1) {
	
	switch(uc1)
	{
		case 0:     // Speed : 0, OFF
			if(!isSleep) {
				set_LED_Fan(false);     Tlc.set(LED_SPEED_LOW, LED_OFF);  Tlc.set(LED_SPEED_MED, LED_OFF);  Tlc.set(LED_SPEED_HIGH, LED_OFF);
			}
			ledStatus &= ~((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH_DIM));
			break;
		case 1:     // Speed : 1
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_DIM);  Tlc.set(LED_SPEED_MED, LED_OFF);  Tlc.set(LED_SPEED_HIGH, LED_OFF);
			}
			ledStatus |= ((1 << LED_SPEED_LOW) | (1 << LED_SPEED_LOW_DIM));
			ledStatus &= ~((1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH_DIM));
			break;
		case 2:     // Speed : 2
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_ON);   Tlc.set(LED_SPEED_MED, LED_OFF);  Tlc.set(LED_SPEED_HIGH, LED_OFF);
			}
			ledStatus |= (1 << LED_SPEED_LOW);
			ledStatus &= ~((1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH) | (1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH_DIM));
			break;
		case 3:     // Speed : 3
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_ON);   Tlc.set(LED_SPEED_MED, LED_DIM);  Tlc.set(LED_SPEED_HIGH, LED_OFF);
			}
			ledStatus |= ((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED) | (1 << LED_SPEED_MED_DIM));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_HIGH) | (1 << LED_SPEED_HIGH_DIM));
			break;
		case 4:     // Speed : 4
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_ON);   Tlc.set(LED_SPEED_MED, LED_ON);   Tlc.set(LED_SPEED_HIGH, LED_OFF);
			}
			ledStatus |= ((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH) | (1 << LED_SPEED_HIGH_DIM));
			break;
		case 5:     // Speed : 5
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_ON);   Tlc.set(LED_SPEED_MED, LED_ON);   Tlc.set(LED_SPEED_HIGH, LED_DIM);
			}
			ledStatus |= ((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH) | (1 << LED_SPEED_HIGH_DIM));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM));
			break;
		case 6:     // Speed : 6
			if(!isSleep) {
				set_LED_Fan(true);      Tlc.set(LED_SPEED_LOW, LED_ON);   Tlc.set(LED_SPEED_MED, LED_ON);   Tlc.set(LED_SPEED_HIGH, LED_ON);
			}
			ledStatus |= ((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH_DIM));
			break;
		default:    // Speed : 0, Unknown input, OFF
			set_LED_Fan(false);     Tlc.set(LED_SPEED_LOW, LED_OFF);  Tlc.set(LED_SPEED_MED, LED_OFF);  Tlc.set(LED_SPEED_HIGH, LED_OFF);
			ledStatus &= ~((1 << LED_SPEED_LOW) | (1 << LED_SPEED_MED) | (1 << LED_SPEED_HIGH));
			ledStatus &= ~((1 << LED_SPEED_LOW_DIM) | (1 << LED_SPEED_MED_DIM) | (1 << LED_SPEED_HIGH_DIM));
			break;    
	}
}

void set_LED_PM01(unsigned char uc1) {
	if(!isSleep) {
		switch(uc1)
		{
			case PM01_LEVEL_GOOD:     // Clean Air
				Tlc.set(LED_PM01_LOW, LED_OFF);  Tlc.set(LED_PM01_MED, LED_OFF);    Tlc.set(LED_PM01_HIGH, LED_OFF);    break;
			case PM01_LEVEL_LOW:     // PM10 Level Low
				Tlc.set(LED_PM01_LOW, LED_ON);   Tlc.set(LED_PM01_MED, LED_OFF);    Tlc.set(LED_PM01_HIGH, LED_OFF);    break;
			case PM01_LEVEL_MED:     // PM10 Level Medium
				Tlc.set(LED_PM01_LOW, LED_ON);   Tlc.set(LED_PM01_MED, LED_ON);     Tlc.set(LED_PM01_HIGH, LED_OFF);    break;
			case PM01_LEVEL_HIGH:     // PM10 Level High
				Tlc.set(LED_PM01_LOW, LED_ON);   Tlc.set(LED_PM01_MED, LED_ON);     Tlc.set(LED_PM01_HIGH, LED_ON);     break;
			default:    // Unknown input, Error State
				Tlc.set(LED_PM01_LOW, LED_OFF);  Tlc.set(LED_PM01_MED, LED_OFF);    Tlc.set(LED_PM01_HIGH, LED_ON);     break;
		}
	}
}

void set_LED_PM25(unsigned char uc1) {
	if(!isSleep) {
		switch(uc1)
		{
			case PM25_LEVEL_GOOD:     // Clean Air
				Tlc.set(LED_PM25_LOW, LED_OFF);  Tlc.set(LED_PM25_MED, LED_OFF);    Tlc.set(LED_PM25_HIGH, LED_OFF);    break;
			case PM25_LEVEL_LOW:     // PM25 Level Low
				Tlc.set(LED_PM25_LOW, LED_ON);   Tlc.set(LED_PM25_MED, LED_OFF);    Tlc.set(LED_PM25_HIGH, LED_OFF);    break;
			case PM25_LEVEL_MED:     // PM25 Level Medium
				Tlc.set(LED_PM25_LOW, LED_ON);   Tlc.set(LED_PM25_MED, LED_ON);     Tlc.set(LED_PM25_HIGH, LED_OFF);    break;
			case PM25_LEVEL_HIGH:     // PM25 Level High
				Tlc.set(LED_PM25_LOW, LED_ON);   Tlc.set(LED_PM25_MED, LED_ON);     Tlc.set(LED_PM25_HIGH, LED_ON);     break;
			default:    // Unknown input, Error State
				Tlc.set(LED_PM25_LOW, LED_OFF);  Tlc.set(LED_PM25_MED, LED_OFF);    Tlc.set(LED_PM25_HIGH, LED_ON);     break;
		}
	}
}

void LEDs_Restore(void)
{
	set_LED_Power((bool)(ledStatus & NUM_LED_POWER));
	set_LED_Fan((bool)(ledStatus & NUM_LED_FAN));
	set_LED_Auto((bool)(ledStatus & NUM_LED_AUTO));
	if(((ledStatus & NUM_LED_SPEED_LOW) == NUM_LED_SPEED_LOW) && ((ledStatus & NUM_LED_SPEED_LOW_DIM) == NUM_LED_SPEED_LOW_DIM))
		Tlc.set(LED_SPEED_LOW, LED_DIM);
	else if(((ledStatus & NUM_LED_SPEED_LOW) == NUM_LED_SPEED_LOW) && ((ledStatus & NUM_LED_SPEED_LOW_DIM) == 0))
		Tlc.set(LED_SPEED_LOW, LED_ON);
	else if(((ledStatus & NUM_LED_SPEED_LOW) == 0) && ((ledStatus & NUM_LED_SPEED_LOW_DIM) == 0))
		Tlc.set(LED_SPEED_LOW, LED_OFF);
	
	if(((ledStatus & NUM_LED_SPEED_MED) == NUM_LED_SPEED_MED) && ((ledStatus & NUM_LED_SPEED_MED_DIM) == NUM_LED_SPEED_MED_DIM))
		Tlc.set(LED_SPEED_MED, LED_DIM);
	else if(((ledStatus & NUM_LED_SPEED_MED) == NUM_LED_SPEED_MED) && ((ledStatus & NUM_LED_SPEED_MED_DIM) == 0))
		Tlc.set(LED_SPEED_MED, LED_ON);
	else if(((ledStatus & NUM_LED_SPEED_MED) == 0) && ((ledStatus & NUM_LED_SPEED_MED_DIM) == 0))
		Tlc.set(LED_SPEED_MED, LED_OFF);
	
	if(((ledStatus & NUM_LED_SPEED_HIGH) == NUM_LED_SPEED_HIGH) && ((ledStatus & NUM_LED_SPEED_HIGH_DIM) == NUM_LED_SPEED_HIGH_DIM))
		Tlc.set(LED_SPEED_HIGH, LED_DIM);
	else if(((ledStatus & NUM_LED_SPEED_HIGH) == NUM_LED_SPEED_HIGH) && ((ledStatus & NUM_LED_SPEED_HIGH_DIM) == 0))
		Tlc.set(LED_SPEED_HIGH, LED_ON);
	else if(((ledStatus & NUM_LED_SPEED_HIGH) == 0) && ((ledStatus & NUM_LED_SPEED_HIGH_DIM) == 0))
		Tlc.set(LED_SPEED_HIGH, LED_OFF);
	
	Tlc.update();
}
