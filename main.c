// tiny2313-hacks-002-adc/main.c

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

void usart_setup()
{
    UBRRH = 0;
    UBRRL = 6;
    UCSRA = 0;
    UCSRB = (1 << TXEN);
    UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}

void usart_tx(unsigned char c)
{
    while ((UCSRA & (1 << UDRE)) == 0x00) {
        // Do nothing
    }
    UDR = c;
}

void usart_tx_string(char* c)
{
    while (*c) {
        usart_tx(*c++);
    }
}

void usart_tx_hex_uint8(uint8_t c)
{
    static char digits[16] = { '0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f' };
    usart_tx(digits[(c >> 4) & 0x0f]);
    usart_tx(digits[(c >> 0) & 0x0f]);
}

void usart_tx_hex_uint16(uint16_t c)
{
    usart_tx_hex_uint8((c >> 8) & 0x00ff);
    usart_tx_hex_uint8((c >> 0) & 0x00ff);
}

//

#define SPI_PORT    DDRB
#define SPI_CLK_PIN PB7
#define SPI_MISO_PIN  PB6
#define SPI_MOSI_PIN  PB5

#define DELAY 0

void spi_setup()
{
    // Clock and Data Out are configured as output pins
    SPI_PORT |= (1 << SPI_CLK_PIN) | (1 << SPI_MOSI_PIN);
}

void spi_write_bits(uint8_t b, uint8_t n)
{
    while (n--) {
        if (b & (1 << n)) {
            PORTB |= (1 << SPI_MOSI_PIN);
        } else {
            PORTB &= ~(1 << SPI_MOSI_PIN);
        }
        PORTB |= (1 << SPI_CLK_PIN);
        PORTB &= ~(1 << SPI_CLK_PIN);
    }
}

uint16_t spi_read_bits(uint8_t n)
{
    uint16_t v = 0;

    for (int i = (n - 1); i >= 0; i--) {
        if (PINB & (1 << SPI_MISO_PIN)) {
            v |= (1 << i);
        }
        PORTB |= (1 << SPI_CLK_PIN);
        PORTB &= ~(1 << SPI_CLK_PIN);
    }
    
    return v;
}

//

#define MCP3002_DDR DDRB
#define MCP3002_OUT PORTB
#define MCP3002_PIN PB0

inline void mcp3002_select()
{
    MCP3002_OUT &= ~(1 << MCP3002_PIN);
}

inline void mcp3002_deselect()
{
    MCP3002_OUT |= (1 << MCP3002_PIN);    
}

void mcp3002_setup()
{
    MCP3002_DDR |= (1 << MCP3002_PIN);
}

uint16_t mcp3002_read(uint8_t channel)
{
    uint16_t value = 0;

    mcp3002_select();
    {
        if (channel == 0) {
            spi_write_bits(0b1101, 4);
        } else {
            spi_write_bits(0b1111, 4);
        }
        value = spi_read_bits(11) & 0b0000001111111111;
    }
    mcp3002_deselect();
    
    return value;
}

//

int main(void)
{
    usart_setup();
    spi_setup();
    mcp3002_setup();

    uint16_t last0 = 0;
    uint16_t last1 = 0;
    uint8_t changed = 0;

    while (1)
    {
        changed = 0;

        uint16_t v0= mcp3002_read(0);
        if (v0 != last0) {
            last0 = v0;
            changed = 1;
        }

        uint16_t v1 = mcp3002_read(1);
        if (v1 != last1) {
            last1 = v1;
            changed = 1;
        }
        
        if (changed) {
            usart_tx_string("Channel #0 = ");
            usart_tx_hex_uint16(v0);
            usart_tx_string(" Channel #1 = ");
            usart_tx_hex_uint16(v1);
            usart_tx('\n');
        }
    }
    
    return 0;
}
