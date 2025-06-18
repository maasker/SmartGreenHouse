
// Microprocessor Project: Smart Greenhouse Watering System

#define _XTAL_FREQ 8000000

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

// CONFIG for PIC16F877A
#pragma config FOSC = HS, WDTE = OFF, PWRTE = OFF, BOREN = OFF, LVP = OFF, CPD = OFF, WRT = OFF, CP = OFF

// I2C LCD ©
#define LCD_ADDR 0x27

// Sensor pins
#define WATER_PIN PORTBbits.RB5      // Water sensor - digital
#define LIGHT_ANALOG_PIN 1           // BFS light sensor on RA1/AN1 - analog
#define SOIL_ANALOG_PIN 2            // Soil moisture sensor on RA2/AN2 - analog

// Pump control
#define PUMP_PIN PORTBbits.RB0       // Pump control on RB0 - digital output
#define PUMP_ON 1
#define PUMP_OFF 0

// Light control - FIXED THRESHOLDS
#define LIGHT_PIN PORTBbits.RB2      // Light control on RB2 - digital output
#define LIGHT_ON 1
#define LIGHT_OFF 0
#define LIGHT_ON_THRESHOLD 40        // Turn ON light when light < 40%
#define LIGHT_OFF_THRESHOLD 80       // Turn OFF light when light >= 80%

// Pump control thresholds
#define SOIL_PUMP_THRESHOLD 30       // Turn on pump if soil moisture below 30%
#define PUMP_MIN_ON_TIME 10          // Minimum pump on time (in 0.5s cycles = 5 seconds)
#define PUMP_MIN_OFF_TIME 60         // Minimum pump off time (in 0.5s cycles = 30 seconds)

// Sensor thresholds (adjust based on your sensors)
#define LIGHT_MAX_READING 1000       // Dark reading (high ADC = dark)
#define LIGHT_MIN_READING 50         // Bright reading (low ADC = bright)
#define SOIL_DRY_READING 800         // Dry soil reading  
#define SOIL_WET_READING 200         // Wet soil reading

// Compile-time safety checks
#if LIGHT_MAX_READING <= LIGHT_MIN_READING
#error "LIGHT_MAX_READING must be greater than LIGHT_MIN_READING"
#endif

#if SOIL_DRY_READING <= SOIL_WET_READING  
#error "SOIL_DRY_READING must be greater than SOIL_WET_READING"
#endif

// ADC Functions for analog sensors
void ADC_Init(void) {
    ADCON0 = 0x41;  // ADC ON, Channel 0 initially, Fosc/8
    ADCON1 = 0x84;  // Right justified, RA0, RA1, RA2 as analog, rest digital
    __delay_us(20); // Allow ADC to settle
}

uint16_t ADC_Read(uint8_t channel) {
    // Validate channel input
    if (channel > 7) return 0;  // Invalid channel protection
    
    ADCON0 &= 0xC7;              // Clear channel selection bits
    ADCON0 |= (channel << 3);    // Select channel
    __delay_us(20);              // Allow settling time
    
    GO_nDONE = 1;                // Start conversion
    uint16_t timeout = 5000;     // ADC timeout protection
    while (GO_nDONE && timeout--) {
        __delay_us(1);
    }
    
    if (timeout == 0) return 0;  // Return 0 if timeout occurred
    
    return ((uint16_t)(ADRESH << 8) + ADRESL);  // Return 10-bit result
}

// I2C Functions with timeout protection
void I2C_Init(void) {
    SSPCON = 0x28;
    SSPCON2 = 0x00;
    SSPADD = 19;
    SSPSTAT = 0x00;
    TRISC3 = 1;
    TRISC4 = 1;
}

void I2C_Wait(void) {
    uint16_t timeout = 1000;  // Prevent infinite loops
    while (((SSPCON2 & 0x1F) || (SSPSTAT & 0x04)) && timeout--) {
        __delay_us(1);
    }
}

void I2C_Start(void) {
    I2C_Wait();
    SEN = 1;
    uint16_t timeout = 1000;  // Timeout protection
    while (SEN && timeout--) {
        __delay_us(1);
    }
}

void I2C_Stop(void) {
    I2C_Wait();
    PEN = 1;
    uint16_t timeout = 1000;  // Timeout protection
    while (PEN && timeout--) {
        __delay_us(1);
    }
}

void I2C_Write(uint8_t data) {
    I2C_Wait();
    SSPBUF = data;
    uint16_t timeout = 1000;  // Timeout protection
    while (!SSPIF && timeout--) {
        __delay_us(1);
    }
    SSPIF = 0;
}

// LCD Functions
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = ((nibble & 0x0F) << 4) | 0x08;  // Mask nibble to avoid warnings
    if (rs) data |= 0x01;
    
    I2C_Start();
    I2C_Write(LCD_ADDR << 1);
    I2C_Write(data | 0x04);
    __delay_us(1);
    I2C_Write(data & ~0x04);
    I2C_Stop();
    __delay_us(50);
}

void LCD_Send_Byte(uint8_t value, uint8_t rs) {
    LCD_Send_Nibble(value >> 4, rs);
    LCD_Send_Nibble(value & 0x0F, rs);
}

void LCD_Cmd(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02)
        __delay_ms(2);
    else
        __delay_us(100);
}

void LCD_Char(char ch) {
    LCD_Send_Byte((uint8_t)ch, 1);
}

void LCD_String(const char *str) {
    while (*str) LCD_Char(*str++);
}

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 1) ? 0x80 + col : 0xC0 + col;
    LCD_Cmd(addr);
}

void LCD_Init(void) {
    __delay_ms(50);
    LCD_Send_Nibble(0x03, 0); __delay_ms(5);
    LCD_Send_Nibble(0x03, 0); __delay_us(150);
    LCD_Send_Nibble(0x03, 0); __delay_us(150);
    LCD_Send_Nibble(0x02, 0); __delay_us(150);
    LCD_Cmd(0x28); LCD_Cmd(0x0C); LCD_Cmd(0x06); LCD_Cmd(0x01); __delay_ms(2);
}

// Pump control function with safety logic
uint8_t Control_Pump(uint8_t water_available, uint8_t soil_percentage) {
    static uint8_t pump_state = PUMP_OFF;
    static uint16_t pump_timer = 0;
    static uint16_t pump_off_timer = 0;
    
    // Safety check: NEVER run pump if no water is available to pump
    if (!water_available) {
        PUMP_PIN = PUMP_OFF;
        pump_state = PUMP_OFF;
        pump_timer = 0;
        pump_off_timer = 0;
        return PUMP_OFF;
    }
    
    // Pump control logic (only runs if water is available)
    if (pump_state == PUMP_OFF) {
        // Pump is currently off
        if (pump_off_timer > 0) {
            pump_off_timer--;  // Still in cooldown period
        } else if (soil_percentage < SOIL_PUMP_THRESHOLD) {
            // Soil is dry and cooldown is over - start pump
            PUMP_PIN = PUMP_ON;
            pump_state = PUMP_ON;
            pump_timer = PUMP_MIN_ON_TIME;
        }
    } else {
        // Pump is currently on
        if (pump_timer > 0) {
            pump_timer--;  // Continue running for minimum time
        } else if (soil_percentage >= SOIL_PUMP_THRESHOLD + 10) {
            // Soil is wet enough (with hysteresis) - stop pump
            PUMP_PIN = PUMP_OFF;
            pump_state = PUMP_OFF;
            pump_off_timer = PUMP_MIN_OFF_TIME;
        }
        // If soil is still dry after minimum time, keep running
    }
    
    return pump_state;
}
// Convert light sensor reading to percentage (with overflow protection and correct inversion)
uint8_t Get_Light_Percentage(uint16_t light_value) {
    uint8_t percentage;
    
    // Clamp input values to prevent overflow
    if (light_value >= LIGHT_MAX_READING) {
        percentage = 0;  // High ADC value = Dark = 0%
    } else if (light_value <= LIGHT_MIN_READING) {
        percentage = 100;  // Low ADC value = Bright = 100%
    } else {
        // Check for division by zero (crash protection)
        uint16_t range = LIGHT_MAX_READING - LIGHT_MIN_READING;
        if (range == 0) {
            percentage = 50;  // Safe fallback if constants are wrong
        } else {
            // INVERTED calculation: higher ADC = darker, so invert the result
            uint32_t temp = ((uint32_t)(light_value - LIGHT_MIN_READING) * 100);
            uint32_t result = temp / range;
            
            // Invert the percentage (100% - calculated value)
            if (result > 100) {
                percentage = 0;  // If calculation error, assume dark
            } else {
                percentage = (uint8_t)(100 - result);  // Invert for correct light reading
            }
        }
    }
    
    return percentage;
}

// Convert soil moisture reading to percentage and status (with overflow protection)
void Get_Soil_Status(uint16_t soil_value, char* status, uint8_t* percentage) {
    // Validate input pointers
    if (status == 0 || percentage == 0) return;
    
    // For soil sensors: higher reading = drier soil, so invert for moisture %
    if (soil_value >= SOIL_DRY_READING) {
        *percentage = 0;  // Very dry
        status[0] = 'D'; status[1] = 'R'; status[2] = 'Y'; status[3] = '\0';  // Safe string copy
    } else if (soil_value <= SOIL_WET_READING) {
        *percentage = 100;  // Very wet
        status[0] = 'W'; status[1] = 'E'; status[2] = 'T'; status[3] = '\0';  // Safe string copy
    } else {
        // Check for division by zero (crash protection)
        uint16_t range = SOIL_DRY_READING - SOIL_WET_READING;
        if (range == 0) {
            *percentage = 50;  // Safe fallback if constants are wrong
            status[0] = 'M'; status[1] = 'O'; status[2] = 'I'; status[3] = '\0';
        } else {
            // Safe calculation with overflow protection
            uint32_t temp = ((uint32_t)(soil_value - SOIL_WET_READING) * 100);
            uint32_t result = temp / range;
            uint32_t moisture_result = 100 - result;  // Invert for moisture percentage
            
            // Prevent uint8_t overflow wraparound and handle underflow
            if (moisture_result > 100) {
                *percentage = 100;  // Clamp to max if calculation error
            } else {
                *percentage = (uint8_t)moisture_result;
            }
            
            // Set status based on moisture level with safe string operations
            if (*percentage >= 70) {
                status[0] = 'W'; status[1] = 'E'; status[2] = 'T'; status[3] = '\0';
            } else if (*percentage >= 40) {
                status[0] = 'M'; status[1] = 'O'; status[2] = 'I'; status[3] = '\0';  // Moist
            } else {
                status[0] = 'D'; status[1] = 'R'; status[2] = 'Y'; status[3] = '\0';
            }
        }
    }
}

// Safe display function with pump AND light status
void Update_Display(uint8_t water_detected, uint16_t light_value, uint16_t soil_value, uint8_t pump_status, uint8_t light_status) {
    char buffer[17];  // 16 chars + null terminator
    char soil_status[4];  // 3 chars + null terminator
    uint8_t light_percentage;
    uint8_t soil_percentage;
    uint8_t i;
    
    // Get sensor percentages and status
    light_percentage = Get_Light_Percentage(light_value);
    Get_Soil_Status(soil_value, soil_status, &soil_percentage);
    
    // Clamp percentages to valid range (extra safety)
    if (light_percentage > 99) light_percentage = 99;
    if (soil_percentage > 99) soil_percentage = 99;
    
    // Line 1: Water, Light percentage, and Light status
    LCD_Set_Cursor(1, 0);
    
    // Clear buffer first
    for (i = 0; i < 16; i++) buffer[i] = ' ';
    buffer[16] = '\0';
    
    // Build string safely: "W:NO L:75% LT:ON"
    buffer[0] = 'W'; buffer[1] = ':';
    if (water_detected) {
        buffer[2] = 'Y'; buffer[3] = 'E'; buffer[4] = 'S';
    } else {
        buffer[2] = 'N'; buffer[3] = 'O'; buffer[4] = ' ';
    }
    buffer[5] = ' '; buffer[6] = 'L'; buffer[7] = ':';
    
    // Add light percentage (max 2 digits)
    if (light_percentage >= 10) {
        buffer[8] = '0' + (light_percentage / 10);
        buffer[9] = '0' + (light_percentage % 10);
    } else {
        buffer[8] = ' ';
        buffer[9] = '0' + light_percentage;
    }
    buffer[10] = '%'; buffer[11] = ' ';
    
    // Add LIGHT status
    buffer[12] = 'L'; buffer[13] = 'T';
    if (light_status == LIGHT_ON) {
        buffer[14] = 'O'; buffer[15] = 'N';
    } else {
        buffer[14] = ' '; buffer[15] = ' ';
    }
    
    LCD_String(buffer);
    
    // Line 2: Soil moisture percentage and PUMP status
    LCD_Set_Cursor(2, 0);
    
    // Clear buffer first
    for (i = 0; i < 16; i++) buffer[i] = ' ';
    buffer[16] = '\0';
    
    // Build string safely: "S:75% WET P:ON"
    buffer[0] = 'S'; buffer[1] = ':';
    
    // Add soil percentage (max 2 digits)
    if (soil_percentage >= 10) {
        buffer[2] = '0' + (soil_percentage / 10);
        buffer[3] = '0' + (soil_percentage % 10);
    } else {
        buffer[2] = ' ';
        buffer[3] = '0' + soil_percentage;
    }
    buffer[4] = '%'; buffer[5] = ' ';
    
    // Add soil status (exactly 3 characters)
    buffer[6] = soil_status[0];
    buffer[7] = soil_status[1];
    buffer[8] = soil_status[2];
    buffer[9] = ' ';
    
    // Add PUMP status on line 2
    buffer[10] = 'P'; buffer[11] = ':';
    if (pump_status == PUMP_ON) {
        buffer[12] = 'O'; buffer[13] = 'N';
    } else {
        buffer[12] = ' '; buffer[13] = ' ';
    }
    
    // Add LOW indicator if needed
    if (soil_percentage < SOIL_PUMP_THRESHOLD && !water_detected && pump_status == PUMP_OFF) {
        buffer[14] = 'L'; buffer[15] = 'W';  // "LW" for LOW
    }
    
    LCD_String(buffer);
}

// Debounce the sensor reading with safety limits
uint8_t Debounce_Sensor(uint8_t current_reading) {
    static uint8_t last_reading = 0;
    static uint8_t stable_count = 0;
    
    // Clamp input to valid values (0 or 1)
    current_reading = (current_reading != 0) ? 1 : 0;
    
    if (current_reading == last_reading) {
        stable_count++;
        if (stable_count >= 3) {  // Sensor must be stable for 3 consecutive readings
            stable_count = 3;  // Cap it to prevent overflow
            return current_reading;  // Return stable reading
        }
    } else {
        stable_count = 0;  // Reset if reading changed
    }
    
    last_reading = current_reading;
    return 255;  // Return 255 to indicate "not yet stable"
}

// light control 
uint8_t Control_Light(uint8_t light_percentage) {
    static uint8_t light_state = LIGHT_OFF;
    static uint16_t light_timer = 0;
    static uint16_t light_off_timer = 0;
    
    if (light_state == LIGHT_OFF) {
        // Light is currently off
        if (light_off_timer > 0) {
            light_off_timer--;  // Still in cooldown period
        } else if (light_percentage < LIGHT_ON_THRESHOLD) {
            // It's dark enough and cooldown is over - turn light ON for 8 seconds
            LIGHT_PIN = LIGHT_ON;
            light_state = LIGHT_ON;
            light_timer = 160;  // 8 seconds (assuming 20 calls per second)
        }
    } else {
        // Light is currently on - ignore sensor, just count down timer
        if (light_timer > 0) {
            light_timer--;  // Continue running for 8 seconds
        } else {
            // 8 seconds expired - turn light OFF
            LIGHT_PIN = LIGHT_OFF;
            light_state = LIGHT_OFF;
            light_off_timer = 20;  // Minimum off time (1 second cooldown)
        }
    }
    
    return light_state;
}

void main(void) {
    // Initialize GPIO with safe defaults
    TRISBbits.TRISB5 = 1;    // RB5 as input (water sensor)
    TRISBbits.TRISB0 = 0;    // RB0 as output (pump control) - FIXED PIN NUMBER
    TRISA1 = 1;              // RA1 as input (light sensor)
    TRISA2 = 1;              // RA2 as input (soil moisture sensor)  
    TRISBbits.TRISB2 = 0;    // RB2 as output (light control)

    PORTB = 0x00;            // Clear PORTB (pump and light start OFF)
    PORTA = 0x00;            // Clear PORTA
    
    // Ensure pump and light are OFF at startup
    PUMP_PIN = PUMP_OFF;
    LIGHT_PIN = LIGHT_OFF;

    // Initialize peripherals with error checking
    I2C_Init();
    ADC_Init();              // Initialize ADC for analog sensors
    __delay_ms(100);
    LCD_Init();
    
    // State tracking variables with safe initialization
    uint8_t current_water_state = 255;  // 255 = unknown/uninitialized
    uint16_t light_reading = 0;
    uint16_t soil_reading = 0;
    uint16_t safety_counter = 0;        // Safety counter to prevent infinite loops
    uint8_t pump_status = PUMP_OFF;
    uint8_t light_status = LIGHT_OFF;   // Track light status
    uint8_t soil_percentage = 0;
    
    // Startup message
    LCD_Set_Cursor(1, 0);
    LCD_String("Smart Watering");
    LCD_Set_Cursor(2, 0);
    LCD_String("System v6.4");
    __delay_ms(2000);
    
    // Get initial stable water reading with timeout
    safety_counter = 0;
    while (current_water_state == 255 && safety_counter < 1000) {  // Max 100 seconds timeout
        uint8_t raw_reading = WATER_PIN;
        current_water_state = Debounce_Sensor(raw_reading);
        __delay_ms(100);
        safety_counter++;
    }
    
    // If we couldn't get stable reading, use default
    if (current_water_state == 255) {
        current_water_state = 0;  // Default to no water detected
    }
    
    // Get initial analog readings with validation
    light_reading = ADC_Read(LIGHT_ANALOG_PIN);
    if (light_reading > 1023) light_reading = 0;  // ADC validation
    
    soil_reading = ADC_Read(SOIL_ANALOG_PIN);
    if (soil_reading > 1023) soil_reading = 0;   // ADC validation
    
    // Get initial soil percentage for pump control
    char temp_status[4];
    Get_Soil_Status(soil_reading, temp_status, &soil_percentage);
    
    // Get initial light percentage and set light control
    uint8_t light_percentage = Get_Light_Percentage(light_reading);
    light_status = Control_Light(light_percentage);
    
    // Display initial state
    Update_Display(current_water_state, light_reading, soil_reading, pump_status, light_status);
    
    // Main loop with safety measures
    while (1) {
        // Read all sensors with validation
        uint8_t raw_water_reading = WATER_PIN;
        uint8_t stable_water_reading = Debounce_Sensor(raw_water_reading);
        
        // Read analog sensors with validation
        light_reading = ADC_Read(LIGHT_ANALOG_PIN);
        if (light_reading > 1023) light_reading = 0;  // Validate ADC reading
        
        soil_reading = ADC_Read(SOIL_ANALOG_PIN);
        if (soil_reading > 1023) soil_reading = 0;   // Validate ADC reading
        
        // Update water state if we have a stable reading
        if (stable_water_reading != 255) {
            current_water_state = stable_water_reading;
        }
        
        // Get light percentage for light control
        uint8_t light_percentage = Get_Light_Percentage(light_reading);
    
        // Control light based on light level with FIXED logic
        light_status = Control_Light(light_percentage);
    
        // Get current soil percentage for pump control
        char temp_status[4];
        Get_Soil_Status(soil_reading, temp_status, &soil_percentage);
        
        // Control pump based on soil moisture and water detection
        pump_status = Control_Pump(current_water_state, soil_percentage);
        
        // Update display with all sensor readings and both pump AND light status
        Update_Display(current_water_state, light_reading, soil_reading, pump_status, light_status);
        
        // Delay before next reading
        __delay_ms(500);  // Update every 500ms
        
        // Reset watchdog if enabled (safety measure)
        // CLRWDT();  // Uncomment if watchdog timer is enabled
    }
}