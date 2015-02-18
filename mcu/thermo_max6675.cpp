#include "thermo_max6675.hh"
#include <LUFA/Drivers/Peripheral/SPI.h>

#define SPI_PORT PORTC
#define SPI_VAR SPIC
#define CS_PORT PORTC
#define CS_PIN 0b00001000;
#define SPI_SS_PIN 0b00010000
#define SPI_MOSI_PIN 0b00100000
#define SPI_SCK_PIN 0b10000000

// http://datasheets.maximintegrated.com/en/ds/MAX6675.pdf

ThermoMax6675::ThermoMax6675()
{
    Setup();
}

bool ThermoMax6675::Setup()
{
    // Setup SPI pins
    SPI_PORT.DIRSET  = SPI_MOSI_PIN | SPI_SCK_PIN | SPI_SS_PIN;
    SPI_PORT.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

    SetChipSelected(false);

    // Set the actual CS pin (which is not in the SPI port)
    CS_PORT.DIRSET = CS_PIN;
    CS_PORT.OUTCLR = CS_PIN;

    // Setup SPI
    // It seems that even SPI_SPEED_FCPU_DIV_4  could be usable
    SPI_Init(&SPI_VAR,
             SPI_SPEED_FCPU_DIV_128  | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_RISING |
             SPI_SAMPLE_LEADING | SPI_MODE_MASTER);

    return 0;
}

void ThermoMax6675::SetChipSelected(const bool selected)
{
    if (selected) {
        CS_PORT.OUTCLR = CS_PIN;
    } else {
        CS_PORT.OUTSET = CS_PIN;
    }
}

uint16_t ThermoMax6675::GetValue()
{
    SetChipSelected(true);
    uint16_t b1 = SPI_ReceiveByte(&SPI_VAR)*255;
    b1 += ((uint16_t)SPI_ReceiveByte(&SPI_VAR));
    b1 = b1>>3;
    b1 /= 4;
    SetChipSelected(false);

    // Must have at least 220ms time before next conversion
    // otherwise nothing will be converted
    _delay_ms(500);
    return b1;
}
