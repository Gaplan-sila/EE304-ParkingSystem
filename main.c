#include "stm32f10x.h"
#include <stdio.h>

// ==============================================================================
// SECTION 1: PIN CONFIGURATION (BASED ON YOUR SCHEMATIC)
// ==============================================================================

// --- ULTRASONIC SENSORS ---
#define PORT_SENSOR   GPIOA
#define TRIG1_PIN     0          // Park 1
#define ECHO1_PIN     1

#define TRIG2_PIN     2          // Park 2
#define ECHO2_PIN     3

#define TRIG3_PIN     4          // Park 3
#define ECHO3_PIN     5

// --- SERVO MOTORS ---
#define PORT_SERVO    GPIOA
#define SERVO_ENTRY   6          // Entry Gate
#define SERVO_EXIT    7          // Exit Gate

// --- IR BARRIER SYSTEM ---
#define PORT_IR_TX    GPIOA
#define IR_TX_PIN     8          // IR LED (PWM Signal)

#define PORT_IR_RX    GPIOB
#define IR_ENTRY_RX   12         // Entry Sensor
#define IR_EXIT_RX    13         // Exit Sensor

// --- LED INDICATORS (MIXED PORTS) ---

// PARK 1 LEDs
#define P1_RED_PORT   GPIOB
#define P1_RED_PIN    0

#define P1_YEL_PORT   GPIOB
#define P1_YEL_PIN    1

#define P1_GRN_PORT   GPIOB
#define P1_GRN_PIN    10

// PARK 2 LEDs
#define P2_RED_PORT   GPIOA      // Red is on PA9
#define P2_RED_PIN    9

#define P2_YEL_PORT   GPIOB      // Yellow is on PB15
#define P2_YEL_PIN    15

#define P2_GRN_PORT   GPIOB      // Green is on PB14
#define P2_GRN_PIN    14

// PARK 3 LEDs
#define P3_RED_PORT   GPIOA
#define P3_RED_PIN    12

#define P3_YEL_PORT   GPIOA
#define P3_YEL_PIN    11

#define P3_GRN_PORT   GPIOA
#define P3_GRN_PIN    10


// ==============================================================================
// SECTION 2: I2C LCD DRIVER (REGISTER LEVEL)
// ==============================================================================


#define LCD_ADDR (0x27 << 1) // Standard I2C Address for LCD

void Delay_us(uint32_t us) {
    volatile uint32_t i;
    for (i = 0; i < (us * 1); i++);
}

void Delay_ms(uint32_t ms) {
    while(ms--) Delay_us(1000);
}

// Initialize I2C1 Hardware
void I2C1_Init(void) {
    // Enable Clocks for GPIOB and I2C1
    RCC->APB2ENR |= (1 << 3);  // GPIOB
    RCC->APB1ENR |= (1 << 21); // I2C1

    // Configure PB6 (SCL) and PB7 (SDA) as Alternate Function Open-Drain
    GPIOB->CRL &= ~(0xFF << 24); // Clear bits for PB6 & PB7
    GPIOB->CRL |= (0xFF << 24);  // Set to 1111 (AF Open-Drain 50MHz)

    // Reset I2C1
    I2C1->CR1 |= (1 << 15);
    I2C1->CR1 &= ~(1 << 15);

    // Set Clock Speed (Standard Mode 100kHz)
    // APB1 is 8MHz.
    I2C1->CR2 = 8;   // Input Clock = 8MHz
    I2C1->CCR = 40;  // 100kHz calculation
    I2C1->TRISE = 9; // Rise time

    // Enable I2C1
    I2C1->CR1 |= (1 << 0);
}

void I2C_Write(uint8_t data) {
    while (!(I2C1->SR1 & (1 << 7))); // Wait for TXE (Transmit Empty)
    I2C1->DR = data;
    while (!(I2C1->SR1 & (1 << 2))); // Wait for BTF (Byte Transfer Finished)
}

void I2C_Start(void) {
    I2C1->CR1 |= (1 << 8); // Generate START
    while (!(I2C1->SR1 & (1 << 0))); // Wait for SB (Start Bit)
}

void I2C_Address(uint8_t addr) {
    I2C1->DR = addr;
    while (!(I2C1->SR1 & (1 << 1))); // Wait for ADDR
    (void)I2C1->SR2; // Clear ADDR flag
}

void I2C_Stop(void) {
    I2C1->CR1 |= (1 << 9); // Generate STOP
}

void LCD_SendInternal(uint8_t data, uint8_t flags) {
    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t data_arr[4];

    // 0x08 is for Backlight ON, 0x04 is Enable bit
    data_arr[0] = up | flags | 0x08 | 0x04;
    data_arr[1] = up | flags | 0x08 | 0x00;
    data_arr[2] = lo | flags | 0x08 | 0x04;
    data_arr[3] = lo | flags | 0x08 | 0x00;

    I2C_Start();
    I2C_Address(LCD_ADDR);
    for(int i=0; i<4; i++) I2C_Write(data_arr[i]);
    I2C_Stop();
}

void LCD_Cmd(uint8_t cmd) { LCD_SendInternal(cmd, 0); Delay_ms(2); }
void LCD_Data(uint8_t data) { LCD_SendInternal(data, 1); Delay_ms(2); }

void LCD_Init(void) {
    I2C1_Init();
    Delay_ms(50);
    // Initialization Sequence
    LCD_Cmd(0x30); Delay_ms(5);
    LCD_Cmd(0x30); Delay_ms(1);
    LCD_Cmd(0x30); Delay_ms(1);
    LCD_Cmd(0x20); // 4-bit mode
    LCD_Cmd(0x28); // Function set
    LCD_Cmd(0x08); // Display off
    LCD_Cmd(0x01); // Clear
    Delay_ms(2);
    LCD_Cmd(0x06); // Entry mode
    LCD_Cmd(0x0C); // Display on
}

void LCD_String(char *str) {
    while (*str) LCD_Data(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_Cmd(addr + col);
}

// Wrapper Function for Main Loop
void LCD_Update_Screen(int free_slots, char* message) {
    char buffer[16];
    LCD_Cmd(0x01); // Clear Screen
    Delay_ms(2);

    LCD_SetCursor(0, 0);
    LCD_String(message);

    LCD_SetCursor(1, 0);
    sprintf(buffer, "Free Spots: %d", free_slots);
    LCD_String(buffer);
}


// ==============================================================================
// SECTION 3: GPIO & PERIPHERAL DRIVERS
// ==============================================================================

void Pin_Mode_Output(GPIO_TypeDef *PORT, int pin) {
    if (pin < 8) {
        PORT->CRL &= ~(0xF << (pin * 4)); PORT->CRL |= (0x2 << (pin * 4));
    } else {
        PORT->CRH &= ~(0xF << ((pin - 8) * 4)); PORT->CRH |= (0x2 << ((pin - 8) * 4));
    }
}

void Pin_Mode_Input_Floating(GPIO_TypeDef *PORT, int pin) {
    if (pin < 8) {
        PORT->CRL &= ~(0xF << (pin * 4)); PORT->CRL |= (0x4 << (pin * 4));
    } else {
        PORT->CRH &= ~(0xF << ((pin - 8) * 4)); PORT->CRH |= (0x4 << ((pin - 8) * 4));
    }
}

void Pin_Mode_Input_PullUp(GPIO_TypeDef *PORT, int pin) {
    if (pin < 8) {
        PORT->CRL &= ~(0xF << (pin * 4)); PORT->CRL |= (0x8 << (pin * 4));
    } else {
        PORT->CRH &= ~(0xF << ((pin - 8) * 4)); PORT->CRH |= (0x8 << ((pin - 8) * 4));
    }
    PORT->ODR |= (1 << pin);
}

void Pin_Mode_AltFunc(GPIO_TypeDef *PORT, int pin) {
    if (pin < 8) {
        PORT->CRL &= ~(0xF << (pin * 4)); PORT->CRL |= (0xB << (pin * 4));
    } else {
        PORT->CRH &= ~(0xF << ((pin - 8) * 4)); PORT->CRH |= (0xB << ((pin - 8) * 4));
    }
}

void System_Init_All(void) {
    // Enable Clocks: AFIO, GPIOA, GPIOB, TIM1
    RCC->APB2ENR |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 11);

    // Sensors
    Pin_Mode_Output(PORT_SENSOR, TRIG1_PIN); Pin_Mode_Input_Floating(PORT_SENSOR, ECHO1_PIN);
    Pin_Mode_Output(PORT_SENSOR, TRIG2_PIN); Pin_Mode_Input_Floating(PORT_SENSOR, ECHO2_PIN);
    Pin_Mode_Output(PORT_SENSOR, TRIG3_PIN); Pin_Mode_Input_Floating(PORT_SENSOR, ECHO3_PIN);

    // Servos
    Pin_Mode_Output(PORT_SERVO, SERVO_ENTRY);
    Pin_Mode_Output(PORT_SERVO, SERVO_EXIT);

    // IR System
    Pin_Mode_AltFunc(PORT_IR_TX, IR_TX_PIN);
    Pin_Mode_Input_PullUp(PORT_IR_RX, IR_ENTRY_RX);
    Pin_Mode_Input_PullUp(PORT_IR_RX, IR_EXIT_RX);

    // LEDs (Custom Pinout)
    Pin_Mode_Output(P1_RED_PORT, P1_RED_PIN);
    Pin_Mode_Output(P1_YEL_PORT, P1_YEL_PIN);
    Pin_Mode_Output(P1_GRN_PORT, P1_GRN_PIN);

    Pin_Mode_Output(P2_RED_PORT, P2_RED_PIN);
    Pin_Mode_Output(P2_YEL_PORT, P2_YEL_PIN);
    Pin_Mode_Output(P2_GRN_PORT, P2_GRN_PIN);

    Pin_Mode_Output(P3_RED_PORT, P3_RED_PIN);
    Pin_Mode_Output(P3_YEL_PORT, P3_YEL_PIN);
    Pin_Mode_Output(P3_GRN_PORT, P3_GRN_PIN);
}

void IR_Start_Signal(void) {
    // 38kHz PWM for 8MHz System Clock
    TIM1->PSC = 0; TIM1->ARR = 210; TIM1->CCR1 = 105;
    TIM1->CCMR1 |= (6 << 4) | (1 << 3);
    TIM1->CCER |= (1 << 0); TIM1->BDTR |= (1 << 15);
    TIM1->CR1 |= (1 << 0);
}

// ==============================================================================
// SECTION 4: LOGIC MODULES
// ==============================================================================

uint32_t Measure_Distance(GPIO_TypeDef* PORT, int trig_pin, int echo_pin) {
    PORT->BSRR = (1 << trig_pin); Delay_us(12); PORT->BRR = (1 << trig_pin);
    uint32_t timeout = 50000;
    while( !(PORT->IDR & (1 << echo_pin)) && timeout-- );
    if (timeout == 0) return 99999;
    uint32_t sure = 0;
    while( (PORT->IDR & (1 << echo_pin)) ) {
        sure++; Delay_us(1);
        if(sure > 25000) break;
    }
    return sure;
}

int Update_Park_Status(uint32_t distance,
                       GPIO_TypeDef* Port_R, int Pin_R,
                       GPIO_TypeDef* Port_Y, int Pin_Y,
                       GPIO_TypeDef* Port_G, int Pin_G)
{
    if (distance < 300) {
        Port_R->BSRR = (1 << Pin_R); Port_Y->BRR  = (1 << Pin_Y); Port_G->BRR  = (1 << Pin_G);
        return 1;
    } else if (distance < 1200) {
        Port_Y->BSRR = (1 << Pin_Y); Port_R->BRR  = (1 << Pin_R); Port_G->BRR  = (1 << Pin_G);
        return 1;
    } else {
        Port_G->BSRR = (1 << Pin_G); Port_R->BRR  = (1 << Pin_R); Port_Y->BRR  = (1 << Pin_Y);
        return 0;
    }
}

void Control_Gate(GPIO_TypeDef* PORT, int pin, int state) {
    int high_time_us = (state == 1) ? 2000 : 1000;
    for(int i=0; i<30; i++) {
        PORT->BSRR = (1 << pin); Delay_us(high_time_us);
        PORT->BRR = (1 << pin);  Delay_us(20000 - high_time_us);
    }
}

// ==============================================================================
// SECTION 5: MAIN PROGRAM
// ==============================================================================

int main(void) {

    System_Init_All();
    IR_Start_Signal();
    LCD_Init(); // Start LCD via I2C

    int free_slots = 3;
    int p1_occ=0, p2_occ=0, p3_occ=0;

    // Initial Message
    LCD_Update_Screen(free_slots, "SYSTEM READY");
    Delay_ms(2000);

    Control_Gate(PORT_SERVO, SERVO_ENTRY, 0);
    Control_Gate(PORT_SERVO, SERVO_EXIT, 0);

    while (1) {

        // 1. Update Park Status
        uint32_t d1 = Measure_Distance(PORT_SENSOR, TRIG1_PIN, ECHO1_PIN);
        p1_occ = Update_Park_Status(d1, P1_RED_PORT, P1_RED_PIN, P1_YEL_PORT, P1_YEL_PIN, P1_GRN_PORT, P1_GRN_PIN);

        uint32_t d2 = Measure_Distance(PORT_SENSOR, TRIG2_PIN, ECHO2_PIN);
        p2_occ = Update_Park_Status(d2, P2_RED_PORT, P2_RED_PIN, P2_YEL_PORT, P2_YEL_PIN, P2_GRN_PORT, P2_GRN_PIN);

        uint32_t d3 = Measure_Distance(PORT_SENSOR, TRIG3_PIN, ECHO3_PIN);
        p3_occ = Update_Park_Status(d3, P3_RED_PORT, P3_RED_PIN, P3_YEL_PORT, P3_YEL_PIN, P3_GRN_PORT, P3_GRN_PIN);

        free_slots = 3 - (p1_occ + p2_occ + p3_occ);
        LCD_Update_Screen(free_slots, "WELCOME");

        // 2. Entry Gate Logic (Active HIGH when beam broken)
        // DÜZELTME: Isimler PORT_IR_RX ve IR_ENTRY_RX olarak degistirildi
        if ( (PORT_IR_RX->IDR & (1 << IR_ENTRY_RX)) != 0 )
        {
            if (free_slots > 0) {
                LCD_Update_Screen(free_slots, "OPENING GATE...");
                Control_Gate(PORT_SERVO, SERVO_ENTRY, 1);
                
                // DÜZELTME: Isimler burada da degistirildi
                while( (PORT_IR_RX->IDR & (1 << IR_ENTRY_RX)) != 0 );
                
                Delay_ms(2000);
                Control_Gate(PORT_SERVO, SERVO_ENTRY, 0);
            } else {
                LCD_Update_Screen(0, "PARKING FULL!");
                Delay_ms(2000);
            }
        }

        // 3. Exit Gate Logic
        // DÜZELTME: Isimler PORT_IR_RX ve IR_EXIT_RX olarak degistirildi
        if ( (PORT_IR_RX->IDR & (1 << IR_EXIT_RX)) != 0 )
        {
            LCD_Update_Screen(free_slots, "GOODBYE");
            Control_Gate(PORT_SERVO, SERVO_EXIT, 1);
            
            // DÜZELTME: Isimler burada da degistirildi
            while( (PORT_IR_RX->IDR & (1 << IR_EXIT_RX)) != 0 );
            
            Delay_ms(2000);
            Control_Gate(PORT_SERVO, SERVO_EXIT, 0);
        }

        Delay_ms(100);
    }}