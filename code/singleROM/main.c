#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/clocks.h"

#include "rom.h"

// Pin Definitions.
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10
#define A11 11
#define A12 12
#define A13 13
#define A14 14
#define A15 15

#define D0 16
#define D1 17
#define D2 18
#define D3 19
#define D4 20
#define D5 21
#define D6 22
#define D7 26

#define NWR 27
#define RST 28

#define DATAOFFSETLOW 16
#define DATAOFFSETHIGH 26

#define LED_INT 25

#define RSTMS 100

// Bit masks.
#define ADDRMASK        0b00000000000000001111111111111111
#define DATAMASK        0b00000100011111110000000000000000
//                                        FEDCBA9876543210
//                        FEDCBA9876543210
#define LOWDATAMASK     0b00000000011111110000000000000000
#define BUS_DATA_BIT_7  0b00000100000000000000000000000000
#define NWRMASK         0b00001000000000000000000000000000
#define RSTMASK         0b00010000000000000000000000000000
#define A15MASK         0b00000000000000001000000000000000

//                        FEDCBA9876543210
//                                        FEDCBA9876543210
#define A15_TO_13_MASK  0b00000000000000001110000000000000
#define SRAM_ADDR_MATCH 0b00000000000000001010000000000000  // If A15=HI then: Cart SRAM addrs uniquely have A14=LO, A13=HI
#define SRAM_RANGE_MASK 0b00000000000000000001111111111111  // 0xBFFF - 0xC000 = 0x1FFF total cart SRAM range
// bank switch reg addr                   0001000000000000
//                       76543210
#define DATA_MASK_LO   0b01111111
#define DATA_MASK_HI   0b10000000
#define RAM_BANKMASK   0b00000011  // Supports banks 0-3 in upper nibble, here it's been downshifted by 4 bits
#define ROM_BANKMASK   0b00001111  // Supports banks 0-15 in lower nybble
#define RAM_BANK_SHIFT 4         // RAM bank bits are in the upper nibble

#define RAM_BANK_COUNT         4
#define RAM_BANK_SIZE          0x2000
#define ROM_BANK_SIZE          0x8000  // 32K bank size, NOT split into 16K

#define MD0_BANK_REGISTER_ADDR 0x1000 

// pico pi 2040: 264kB of SRAM, 2MB of flash storage

uint8_t cart_sram[RAM_BANK_COUNT * RAM_BANK_SIZE]; // 4 x 8k = 32K Cart SRAM 

void initGPIO() {
  // Set all pins to input first.
  gpio_init_mask( ADDRMASK | DATAMASK | NWRMASK );
  gpio_set_dir_in_masked( ADDRMASK | DATAMASK | NWRMASK );
  
  // Except the LED.
  gpio_init( LED_INT );
  gpio_set_dir( LED_INT, GPIO_OUT );
  
  // And reset.
  // gpio_init( RST );
  // gpio_set_dir( RST, GPIO_OUT );
  gpio_set_dir( RST, GPIO_IN );
}



#define RAM_BANK(N) (cart_sram + (N * RAM_BANK_SIZE))

unsigned char * const p_rambank_offsets[] = {
    RAM_BANK(0),  // 8k
    RAM_BANK(1),  // 16k
    RAM_BANK(2),  // 24k
    RAM_BANK(3),  // 32k
};


#define ROM_BANK(N) (rom + (N * ROM_BANK_SIZE))

const unsigned char * p_rombank_offsets[] = {
    ROM_BANK(0),  ROM_BANK(1),  // 64K
    ROM_BANK(2),  ROM_BANK(3),  // 128K
    ROM_BANK(4),  ROM_BANK(5),  
    ROM_BANK(6),  ROM_BANK(7),  // 256K
    ROM_BANK(8),  ROM_BANK(9),
    ROM_BANK(10), ROM_BANK(11),
    ROM_BANK(12), ROM_BANK(13),
    ROM_BANK(14), ROM_BANK(15), // 512K
};


void __not_in_flash_func( handleROM_MD0_with_SRAM() ) {
    // Initial bank, point it to BANK 0 for both
    uint8_t * rambank = RAM_BANK(0);       // SRAM
    const uint8_t * rombank = ROM_BANK(0); // ROM

    // Start endless loop
    while( 1 ) {

        uint32_t bus_data = gpio_get_all();
        uint32_t wr = !( bus_data & NWRMASK );
        uint32_t a15 = ( bus_data & A15MASK );

        // All ROM Area 0x0000 - 0x7FFF addresses will have A15 line low
        if (!a15) {

            // ROM Memory region
            uint32_t addr = bus_data & ADDRMASK;

            if (wr) {
                // Data input
                gpio_set_dir_in_masked( DATAMASK );

                // Handle MD0 style bank switch register write at address 0x1000
                if (addr == MD0_BANK_REGISTER_ADDR) {
                    // Remap rambank to requested slice of cart SRAM buffer
                    // RAM bank is in upper 4 bits of data byte (but seems limited to values 0-3)
                    rambank = p_rambank_offsets[(bus_data >> (DATAOFFSETLOW + RAM_BANK_SHIFT)) & RAM_BANKMASK];
                    // ROM bank is in lower 4 bits of data byte, value range 0-15
                    rombank = p_rombank_offsets[(bus_data >> (DATAOFFSETLOW )) & ROM_BANKMASK];
                }
            } else {
              // Data output
              gpio_set_dir_out_masked( DATAMASK );
              
              // Get data byte based on whether it's in upper or lower 16K rom bank region.
              uint8_t rombyte = rom[addr];
              
              uint32_t gpiobyte  = ( rombyte & DATA_MASK_LO ) << DATAOFFSETLOW;
                       gpiobyte |= ( ( rombyte & DATA_MASK_HI ) >> 7 ) << DATAOFFSETHIGH;
              
              // And put it out
              gpio_put_all( gpiobyte );                
            }
        }
        else {
            // Non-ROM Memory region

            // Check to see if it's a Cart SRAM region
            // Don't have the CS pin available so have to test it this way
            if ((bus_data & A15_TO_13_MASK) == SRAM_ADDR_MATCH) {
                // Cart SRAM Memory Region 0xA000 - 0xBFFF
                uint32_t cart_sram_relative_addr = (bus_data & SRAM_RANGE_MASK);

                if (wr) {
                    // Data input
                    gpio_set_dir_in_masked( DATAMASK );                    

                    uint8_t gpiobyte_in = (uint8_t)(bus_data >> DATAOFFSETLOW);
                    // Handle non-contiguous high bit
                    if (bus_data & BUS_DATA_BIT_7) gpiobyte_in |= DATA_MASK_HI;

                    // Write the data to the RAM buffer
                    rambank[cart_sram_relative_addr] = gpiobyte_in;

                }
                else {
                    // Data output
                    gpio_set_dir_out_masked( DATAMASK );

                    uint8_t rambyte = rambank[cart_sram_relative_addr];

                    uint32_t gpiobyte_out = (rambyte & DATA_MASK_LO) << DATAOFFSETLOW;
                    // Handle non-contiguous high bit
                    if (rambyte & DATA_MASK_HI) gpiobyte_out |= BUS_DATA_BIT_7;

                    // Put data byte out on bus
                    gpio_put_all( gpiobyte_out );  // This causes the crash, presumably related to gpio_set_dir_out_masked() above
                } 
            }
            else {
                // If not cart SRAM access then revert pins to input
                gpio_set_dir_in_masked( DATAMASK );
            }
        }  // End: Non-ROM memory region
    }  // End: while(1)
}

void main() {
  // Set higher freq.
  set_sys_clock_khz(250000, true);
  
  // Init GPIO.
  initGPIO();
  
  // Turn on LED.
  gpio_put( LED_INT, 1 );
  
  // Reset.
  gpio_put( RST, 1 );
  sleep_ms( RSTMS );
  gpio_put( RST, 0 );
  
  // Set RST pin back to INPUT, so we can use GPIO_PUT as non-masked.
  gpio_set_dir( RST, GPIO_IN );
  gpio_pull_down( RST );
  
  handleROM_MD0_with_SRAM();

}