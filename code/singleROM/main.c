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
#define ADDRMASK    0b00000000000000001111111111111111
#define DATAMASK    0b00000100011111110000000000000000
//                                    FEDCBA9876543210
//                    FEDCBA9876543210
#define LOWDATAMASK 0b00000000011111110000000000000000
#define NWRMASK     0b00001000000000000000000000000000
#define RSTMASK     0b00010000000000000000000000000000
#define A15MASK     0b00000000000000001000000000000000

//                     76543210
#define DATAMASKLOW  0b01111111
#define DATAMASKHIGH 0b10000000
#define BANKMASK     0b00011111
// #define BANKMASK     0b00000111

// Bank switch
// 1: Dynamic 32 kB banks (e.g., Puppet Knight)
// 2: Fixed 16 kB bank and dynamic 16 kB bank (e.g., Armour Force)
unsigned int BANKSWITCH = 1;

void initGPIO() {
  // Set all pins to input first.
  gpio_init_mask( ADDRMASK | DATAMASK | NWRMASK );
  gpio_set_dir_in_masked( ADDRMASK | DATAMASK | NWRMASK );
  
  // Except the LED.
  gpio_init( LED_INT );
  gpio_set_dir( LED_INT, GPIO_OUT );
  
  // And reset.
  gpio_init( RST );
  gpio_set_dir( RST, GPIO_OUT );
}

#define ROM_BANK(N) (rom + ((N-1) * 0x4000u)) // The reason for N-1 is because rombank will be accessed with a base of (0x4000) + the bank relative address

// pico pi 2040: 264kB of SRAM, 2MB of flash storage

void __not_in_flash_func( handleROM() ) {
  // Initial bank, point it to BANK 1
  const uint8_t * rombank = ROM_BANK(1); // rom;
  
  // Start endless loop.
  while( 1 ) {
    uint32_t data = gpio_get_all();
    uint32_t addr = data & ADDRMASK;
    uint32_t wr = !( data & NWRMASK );
    uint32_t a15 = ( data & A15MASK );
    
    if ( !a15 && !wr ) {
      // Data output.
      gpio_set_dir_out_masked( DATAMASK );
      
      // Get data byte.
      uint8_t rombyte;
      if ( BANKSWITCH == 1 ) {
        rombyte = rombank[ addr ];
        
      } else if ( BANKSWITCH == 2 ) {
        // Check if lower part
        if ( addr < 16384 ) {
          rombyte = rom[ addr ];
        } else {
          rombyte = rombank[ addr ];
        }
      }
      
      uint32_t gpiobyte = 0;
      gpiobyte |= ( ( rombyte & DATAMASKLOW ) >> 0 ) << DATAOFFSETLOW;
      gpiobyte |= ( ( rombyte & DATAMASKHIGH ) >> 7 ) << DATAOFFSETHIGH;
      
      // And put it out.
      //gpio_put_masked( DATAMASK, gpiobyte );
      gpio_put_all( gpiobyte );
      
    } else {
      // Data input.
      gpio_set_dir_in_masked( DATAMASK );
    }
    
    if ( wr ) {
      uint32_t writeData;
      // Bank change?
      switch ( addr ) {
        case 0x000:
          // Bank switch type 1
          BANKSWITCH = 1;
          
          writeData = ( data & LOWDATAMASK ) >> DATAOFFSETLOW;
          if ( writeData == 0 ) {
            rombank = rom;
          } else if ( writeData == 1 ) {
            rombank = rom + 32768;
          }
          break;
          
        case 0x001:
          // Bank switch type 2
          BANKSWITCH = 2;
          
          writeData = ( data & LOWDATAMASK ) & ( BANKMASK << DATAOFFSETLOW );

          // Use a case structure instead of a dynamic multiplcation
          // to hopefully speed up things.
          switch ( writeData ) {
            case 0:
            case ( 1  << DATAOFFSETLOW ): rombank = ROM_BANK(1); break;
            // End 32K
            // 
            case ( 2  << DATAOFFSETLOW ): rombank = ROM_BANK(2); break;
            case ( 3  << DATAOFFSETLOW ): rombank = ROM_BANK(3); break;
            // End 64K
            
            case ( 4  << DATAOFFSETLOW ): rombank = ROM_BANK(4); break;
            case ( 5  << DATAOFFSETLOW ): rombank = ROM_BANK(5); break;
            case ( 6  << DATAOFFSETLOW ): rombank = ROM_BANK(6); break;
            case ( 7  << DATAOFFSETLOW ): rombank = ROM_BANK(7); break;
            // End 128K
            
            case ( 8  << DATAOFFSETLOW ): rombank = ROM_BANK(8); break;
            case ( 9  << DATAOFFSETLOW ): rombank = ROM_BANK(9); break;
            case ( 10 << DATAOFFSETLOW ): rombank = ROM_BANK(10); break;
            case ( 11 << DATAOFFSETLOW ): rombank = ROM_BANK(11); break;
            case ( 12 << DATAOFFSETLOW ): rombank = ROM_BANK(12); break;
            case ( 13 << DATAOFFSETLOW ): rombank = ROM_BANK(13); break;
            case ( 14 << DATAOFFSETLOW ): rombank = ROM_BANK(14); break;
            case ( 15 << DATAOFFSETLOW ): rombank = ROM_BANK(15); break;            
            // End 256K (seems ok)

            case ( 16 << DATAOFFSETLOW ): rombank = ROM_BANK(16); break;
            case ( 17 << DATAOFFSETLOW ): rombank = ROM_BANK(17); break;
            case ( 18 << DATAOFFSETLOW ): rombank = ROM_BANK(18); break;
            case ( 19 << DATAOFFSETLOW ): rombank = ROM_BANK(19); break;
            case ( 20 << DATAOFFSETLOW ): rombank = ROM_BANK(20); break;
            case ( 21 << DATAOFFSETLOW ): rombank = ROM_BANK(21); break;
            case ( 22 << DATAOFFSETLOW ): rombank = ROM_BANK(22); break;
            case ( 23 << DATAOFFSETLOW ): rombank = ROM_BANK(23); break;
            
            case ( 24 << DATAOFFSETLOW ): rombank = ROM_BANK(24); break;
            case ( 25 << DATAOFFSETLOW ): rombank = ROM_BANK(25); break;
            case ( 26 << DATAOFFSETLOW ): rombank = ROM_BANK(26); break;
            case ( 27 << DATAOFFSETLOW ): rombank = ROM_BANK(27); break;
            case ( 28 << DATAOFFSETLOW ): rombank = ROM_BANK(28); break;
            case ( 29 << DATAOFFSETLOW ): rombank = ROM_BANK(29); break;
            case ( 30 << DATAOFFSETLOW ): rombank = ROM_BANK(30); break;
            case ( 31 << DATAOFFSETLOW ): rombank = ROM_BANK(31); break;
            // End 512K (seems ok)


            // Something went wrong.
            default:
              break;
          }
          
          break;
      }
    }
  }
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
  
  handleROM();

}
