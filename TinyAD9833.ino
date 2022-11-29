// serial interface to the AD9833 DDS using a digispark AtTiny85

#include <DigiCDC.h>


//-----------------------------------------------------------------------------
// Main routines
// The setup function
//-----------------------------------------------------------------------------
void setup() {
    initSigGen();
    SerialUSB.begin();
    while ( !SerialUSB )
        ; // wait
    AD9833_reset();
}


// the loop routine runs over and over again forever:
void loop() { parseCommand(); }


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

const byte numberOfDigits = 8;                                  // number of digits in the frequency value
byte digitArray[ numberOfDigits ] = { 0, 0, 0, 1, 0, 0, 0, 0 }; // 1000Hz start frequency

long unsigned multiplier = 1; // set by suffix 'k' and 'M'
long unsigned divider = 1;    // set by decimal point

int8_t debug = 0;
int8_t echo = 1;

/******************************************************************************
    AD9833 register (16 bit)
    D15 D14   00: CONTROL (14 bits)
            01: FREQ0   (14 bits)
            10: FREQ1   (14 bits)
            11: PHASE   (12 bits) (D13 D12 0X: PHASE0, 1X: PHASE1)

    CONTROL bits:

    D13: B28
    Two write operations are required to load a complete word into either of the frequency registers.
    B28 = 1 allows a complete word to be loaded into a frequency register in two consecutive writes.
    The first write contains the 14 LSBs of the frequency word, and the next write contains the 14 MSBs.
    The first two bits of each 16-bit word define the frequency register to which the word is loaded,
    and should therefore be the same for both of the consecutive writes.
    See Table 8 for the appropriate addresses.
    The write to the frequency register occurs after both words have been loaded; therefore, the register
    never holds an intermediate value. An example of a complete 28-bit write is shown in Table 9.
    When B28 = 0, the 28-bit frequency register operates as two 14-bit registers, one containing
    the 14 MSBs and the other containing the 14 LSBs. This means that the 14 MSBs of the frequency word
    can be altered independent of the 14 LSBs, and vice versa. To alter the 14 MSBs or the 14 LSBs,
    a single write is made to the appropriate frequency address. The control bit D12 (HLB) informs
    the AD9833 whether the bits to be altered are the 14 MSBs or 14 LSBs.

    D12: HLB
    This control bit allows the user to continuously load the MSBs or LSBs of a frequency register while
    ignoring the remaining 14 bits. This is useful if the complete 28-bit resolution is not required.
    HLB is used in conjunction with D13 (B28). This control bit indicates whether the 14 bits being
    loaded are being transferred to the 14 MSBs or 14 LSBs of the addressed frequency register.
    D13 (B28) must be set to 0 to be able to change the MSBs and LSBs of a frequency word separately.
    When D13 (B28) = 1, this control bit is ignored. HLB = 1 allows a write to the 14 MSBs of the addressed
    frequency register. HLB = 0 allows a write to the 14 LSBs of the addressed frequency register.

    D11: FSELECT
    The FSELECT bit defines whether the FREQ0 register or the FREQ1 register is used in the phase accumulator.

    D10: PSELECT
    The PSELECT bit defines whether the PHASE0 register or the PHASE1 register data is added to the output
    of the phase accumulator.

    D9: Reserved
    This bit should be set to 0.

    D8: Reset
    Reset = 1 resets internal registers to 0, which corresponds to an analog output of midscale.
    Reset = 0 disables reset.

    D7: SLEEP1
    When SLEEP1 = 1, the internal MCLK clock is disabled, and the DAC output remains at its present value
    because the NCO is no longer accumulating. When SLEEP1 = 0, MCLK is enabled.

    D6: SLEEP12
    SLEEP12 = 1 powers down the on-chip DAC. This is useful when the AD9833 is used to output the MSB
    of the DAC data. SLEEP12 = 0 implies that the DAC is active.

    D5: OPBITEN
    The function of this bit, in association with D1 (mode), is to control what is output at the VOUT pin.
    When OPBITEN = 1, the output of the DAC is no longer available at the VOUT pin. Instead, the MSB
    (or MSB/2) of the DAC data is connected to the VOUT pin. This is useful as a coarse clock source.
    The DIV2 bit controls whether it is the MSB or MSB/2 that is output. When OPBITEN = 0, the DAC is
    connected to VOUT. The mode bit determines whether it is a sinusoidal or a ramp output that is available.

    D4: Reserved
    This bit must be set to 0.

    D3: DIV2
    DIV2 is used in association with D5 (OPBITEN). When DIV2 = 1, the MSB of the DAC data is passed
    directly to the VOUT pin. When DIV2 = 0, the MSB/2 of the DAC data is output at the VOUT pin.

    D2: Reserved
    This bit must be set to 0.

    D1: MODE
    This bit is used in association with OPBITEN (D5). The function of this bit is to control
    what is output at the VOUT pin when the on-chip DAC is connected to VOUT. This bit should
    be set to 0 if the control bit OPBITEN = 1. When MODE = 1, the SIN ROM is bypassed, resulting
    in a triangle output from the DAC. When MODE = 0, the SIN ROM is used to convert the phase
    information into amplitude information, which results in a sinusoidal signal at the output.

    D0: Reserved
    This bit must be set to 0.

*******************************************************************************/

// typical control register values
const word wReset = 0b0000000100000000;      // analog output of midscale (330 mV)
const word wSine = 0b0000000000000000;       // sine 0.6 Vpp
const word wTriangle = 0b0000000000000010;   // triangle 0.6 Vpp
const word wRectangle = 0b0000000000101000;  // rectangle TTL level (0V/5V)
const word wRectangle2 = 0b0000000000100000; // rectangle half frequency TTL

word waveType = wReset;

//-----------------------------------------------------------------------------
// initSigGen
//-----------------------------------------------------------------------------
void initSigGen( void ) {
    DDRB |= _BV( PB0 ) | _BV( PB1 ) | _BV( PB2 ); // PB0,1,2 output
    FSYNC_HIGH();
    CLK_HIGH();
    AD9833_reset();
}


// forward declaration with default argument
void AD9833_setFrequency( bool isFreq = true );

//-----------------------------------------------------------------------------
// parseCommand
//   if a numeric digit is available in the serial input buffer
//   build up a numeric value or execute a command
//-----------------------------------------------------------------------------
void parseCommand( void ) {
    static byte numeric = 0; // number of numeric digits
    static byte decimal = 0; // number of decimal digits

    if ( SerialUSB.available() > 0 ) {
        char c = SerialUSB.read();
        if ( echo && c >= ' ' )
            SerialUSB.write( c );

        // collect up to 8 digits
        if ( ( c >= '0' ) && ( c <= '9' ) ) {
            // erase the array if this is the 1st input digit
            // else shift one position to the right
            for ( byte i = numberOfDigits - 1; i > 0; i-- )
                digitArray[ i ] = numeric ? digitArray[ i - 1 ] : 0;
            // put the new digit in 1st position
            digitArray[ 0 ] = c - '0';
            ++numeric;
            multiplier = 1;
            divider = 1;
        } else if ( c == '.' ) {        // take care of decimal point
            decimal = numeric + 1;      // position _before_ nth digit to catch input like ".123" -> numeric = 3, decimal = 1
        } else {                        // all other char terminate numeric input
            if ( decimal && numeric ) { // digit(s) and decimal point exist
                divider = powerOf10( numeric - decimal + 1 );
            }
            numeric = 0;
            decimal = 0;
            // execute some commands
            switch ( toupper( c ) ) {
            case '?': // show help
                SerialUSB.println();
                showHelp();
                break;
            case 'E':
                SerialUSB.println();
                echo = *digitArray; // lowest digit
                break;
            case 'K': // kilo Hz
                multiplier = 1000;
                break;
            case 'M': // Mega Hz
                multiplier = 1000000;
                break;
            case 'N': // TEST, send number directly to the freq register of AD9833
                SerialUSB.println();
                waveType = wSine;             // SigGen wave is sine
                AD9833_setFrequency( false ); // do not convert freq -> reg, 1->0.0931 Hz
                break;
            case 'O': // Off (Reset)
                SerialUSB.println();
                AD9833_reset();
                break;
            case 'S': // Sine
                waveType = wSine;
                SerialUSB.println();
                AD9833_setFrequency(); // SigGen wave is sine
                break;
            case 'T': // Triangle
                SerialUSB.println();
                waveType = wTriangle;
                AD9833_setFrequency(); // SigGen wave is triangle
                break;
            // disable rect due to too high output voltage / power (AC: 20 dBm)
            // case 'R': // Rectangle
            //    waveType = wRectangle;
            //    AD9833_setFrequency(); // SigGen wave is rectangle
            //    break;
            case 'Z':
                SerialUSB.println();
                debug = *digitArray; // lowest digit
                break;
            default:
                return;
            }
        }
    } // if ( SerialUSB.available() > 0 )
}


void showHelp() {
    SerialUSB.println( F( "TinyAD9833 Frequency Generator" ) );
    SerialUSB.println( F( "usage: <number>[STONED]" ) );
    SerialUSB.println( F( "<number>: 1..8 digits, opt. dot, k, M" ) );
    SerialUSB.println( F( "S:Sin, T:Tri, O:Off" ) );
    SerialUSB.println( F( "N: to reg, n*0.093 Hz Sin" ) );
    SerialUSB.println( F( "E: echo" ) );
    SerialUSB.println( F( "Z: debug" ) );
}


//-----------------------------------------------------------------------------
// calculate the value from the input char array
//-----------------------------------------------------------------------------
double parseValue() {
    unsigned long f = 0;
    for ( byte iii = 0; iii < numberOfDigits; ++iii ) {
        f = f + digitArray[ iii ] * powerOf10( iii );
    }
    return double( f ) * multiplier / divider;
}


//-----------------------------------------------------------------------------
// returns y = 10 ^ x
//-----------------------------------------------------------------------------
unsigned long powerOf10( byte x ) {
    unsigned long y = 1;
    while ( x-- )
        y = y * 10;
    return y;
}


//-----------------------------------------------------------------------------
// AD9833_reset
//-----------------------------------------------------------------------------
void AD9833_reset() {
    AD9833_writeRegister( 0x2100 ); // Control Register (B28 = 1, Reset = 1)
    // AD9833_writeRegister( 0xC000 ); // Phase Register (set to zero)
}


//-----------------------------------------------------------------------------
// AD9833_setFrequency
//    set the frequency and wave type
//-----------------------------------------------------------------------------
void AD9833_setFrequency( bool isFreq ) {
    double frequency = parseValue();
    unsigned long regFrequency;
    if ( isFreq ) {
        frequency *= ( 0x10000000UL / 25000000.0 );
        if ( wRectangle == waveType && frequency < 12e6 ) {
            waveType = wRectangle2;
            frequency *= 2;
        }
    }
    regFrequency = round( frequency );
    if ( regFrequency >= 0x8000000UL )
        regFrequency = 0x7FFFFFFUL;
    AD9833_writeRegister( word( regFrequency & 0x3FFFUL ) | 0x4000UL );            // Frequency Register LSB
    AD9833_writeRegister( word( ( regFrequency & 0xFFFC000UL ) >> 14 ) | 0x4000 ); // Frequency Register MSB
    AD9833_writeRegister( 0x2000 | waveType );                                     // Control Register (B28 = 1, Reset = 0)
}


//-----------------------------------------------------------------------------
// AD9833_writeRegister
//-----------------------------------------------------------------------------
void AD9833_writeRegister( word dat ) {
    if ( debug ) {
        SerialUSB.println( dat, HEX );
    }
    CLK_HIGH();
    FSYNC_LOW();
    for ( byte iii = 0; iii < 16; ++iii ) {
        if ( dat & 0x8000 )
            DATA_HIGH();
        else
            DATA_LOW();
        dat = dat << 1;
        CLK_LOW();
        CLK_HIGH();
    }
    FSYNC_HIGH();
}


//-----------------------------------------------------------------------------
// bit banging procedures
//-----------------------------------------------------------------------------
inline void DATA_HIGH() { PORTB |= _BV( PB0 ); }
inline void DATA_LOW() { PORTB &= ~_BV( PB0 ); }
inline void FSYNC_HIGH() { PORTB |= _BV( PB1 ); }
inline void FSYNC_LOW() { PORTB &= ~_BV( PB1 ); }
inline void CLK_HIGH() { PORTB |= _BV( PB2 ); }
inline void CLK_LOW() { PORTB &= ~_BV( PB2 ); }
