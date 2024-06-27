#define PIEZO_BUZZER_PIN 9

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

// Function to receive data via UART
uint8_t USART_Frame_Receive() {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

void piezoSetup() {
    pinMode(PIEZO_BUZZER_PIN, OUTPUT);
}

void setup() {
    Serial.begin(9600); // For debugging
    piezoSetup();
    USART_Init(MYUBRR);
}

bool alarmActive = false;
unsigned long previousMillis = 0;
const long interval = 5; // Interval at which to increase/decrease frequency

int freq = 1000;
bool increasing = true;

void executeAlarm() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (increasing) {
            freq += 10;
            if (freq >= 2000) {
                increasing = false;
            }
        } else {
            freq -= 10;
            if (freq <= 1000) {
                increasing = true;
            }
        }
        tone(PIEZO_BUZZER_PIN, freq);
    }
}

void stopAlarm() {
    noTone(PIEZO_BUZZER_PIN);
}

void loop() {
    //if (Serial.available() > 0) {
        uint8_t received_data = USART_Frame_Receive();
        Serial.print("Received: "); // Debugging line
        Serial.println(received_data); // Debugging line
        if (received_data == 'G') {
            alarmActive = true;
        } else if (received_data == 'S') {
            alarmActive = false;
            stopAlarm(); // Ensure alarm stops immediately
        }
    //}

    if (alarmActive) {
        executeAlarm();
    } else {
        stopAlarm();
    }
}
