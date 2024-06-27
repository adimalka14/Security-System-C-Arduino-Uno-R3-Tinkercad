#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define ECHO_PIN PB3
#define TRIG_PIN PB4
#define SERVO_PIN PB5
#define RED_LED_PIN PC1
#define GREEN_LED_PIN PC3
#define BLUE_LED_PIN PC2

LiquidCrystal_I2C lcd(0x20, 16, 2);
Servo myServo;

const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {10, 9, 8, 7}; 
byte colPins[COLS] = {6, 5, 4, 3}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// External Oscillator Frequency
#define F_CPU 16000000UL

// Desired Baud Rate
#define BAUD 9600

// Calculating of setting Register to specify corresponding baud rate
#define MYUBRR ((F_CPU / 16 / BAUD) - 1)

// Function to initialize UART
void USART_Init(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Function to transmit data via UART
void USART_Frame_Transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void lcdSetup() {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Enter password:");
    lcd.setCursor(0, 1);
}

void ultrasonicSetup() {
    DDRB |= (1 << TRIG_PIN);
    DDRB &= ~(1 << ECHO_PIN);
}

void rgbLedSetup() {
    DDRC |= (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN);
    PORTC &= ~((1 << RED_LED_PIN) | (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN)); 
}

void buttonSetup() {
    cli();
    DDRD &= ~(1 << PD2);
    EICRA |= (1 << ISC00) | (1 << ISC01);
    EIMSK |= (1 << INT0); 
    
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 124; // Set compare match register for 1kHz increments
    TCCR2A |= (1 << WGM21); // CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
    TIMSK2 |= (1 << OCIE2A); // Enable timer compare interrupt
    sei();
}

void setup() {
    Serial.begin(9600);
    lcdSetup();
    ultrasonicSetup();
    rgbLedSetup();
    buttonSetup();
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    USART_Init(MYUBRR);
}

unsigned int timerCount = 0;
bool buttonPressed = false;
String enteredPassword = "";  
const String correctPassword = "1234";  
int buttonPressCount = 0;
bool lockedUp = true;
bool executeAlarm = false;

ISR(INT0_vect) {
    buttonPressed = true;
    buttonPressCount++;
}

ISR(TIMER2_COMPA_vect) {
    if (buttonPressCount > 0 && lockedUp == true) {
        timerCount++;
        if (buttonPressCount == 1 && timerCount >= 125) {
            PORTC ^= (1 << RED_LED_PIN);
            timerCount = 0;
        } else if (buttonPressCount == 2 && timerCount >= 64) {
            PORTC ^= (1 << RED_LED_PIN);
            timerCount = 0;
        } else if (buttonPressCount > 2 && timerCount >= 32) {
            PORTC ^= (1 << RED_LED_PIN);
            timerCount = 0;
        }
    }
}
                
void loop() {
    char key = keypad.getKey();
    if (key != NO_KEY) {
        lcd.print(key);
        enteredPassword += key;
    }
  
    if (buttonPressed == true) {      
        if (enteredPassword == correctPassword) {
            if (lockedUp == true) {
                unlock();
                lockedUp = false;
            } else {
                lockedUp = true;
                lock();
            }
            buttonPressCount = 0;
        } else {
            if (lockedUp == true && buttonPressCount >= 3) {
                executeAlarm = true;
            }
        }
      
        enteredPassword = "";
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter password:");
        lcd.setCursor(0, 1);
        buttonPressed = false;
    }
  
    if (executeAlarm == true) {
        USART_Frame_Transmit('G');
    } else {
        USART_Frame_Transmit('S');
    }
  
    int distance = measureDistance();
    Serial.print("{distance:");
    Serial.print(distance);
    Serial.println("}");
}

void unlock() {
    USART_Frame_Transmit('S');
    for (int i = 0; i <= 90; i++) {
        myServo.write(i);
        delay(10);
    }
  
    PORTC &= ~(1 << RED_LED_PIN);
    PORTC |= (1 << GREEN_LED_PIN);
    executeAlarm = false;
}

void lock() {
    for (int i = 90; i >= 0; i--) {
        myServo.write(i);
        delay(10);
    }
  
    PORTC |= (1 << RED_LED_PIN);
    PORTC &= ~(1 << GREEN_LED_PIN);
}

long measureDistance() {
    PORTB &= ~(1 << TRIG_PIN);
    delayMicroseconds(2);
    PORTB |= (1 << TRIG_PIN);
    delayMicroseconds(10);
    PORTB &= ~(1 << TRIG_PIN);
    long duration = pulseIn(PB3, HIGH); // Use the actual pin number
    long distance = (duration / 2) / 29.1;
    return distance;
}
