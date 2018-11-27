#include <msp430.h>
#include <math.h>

// Reading the temperature sensor
#define ADC_MAX 4095.0
#define RREF 10000.0
#define CONNECTED_RESISTOR 10000.0
#define A1 3.354016e-3
#define B1 2.56985e-4
#define C1 2.620131e-6
#define D1 6.383091e-8

// Control of the fan
#define K 2.0
#define PWM_OFF 0
#define PWM_MIN 15
#define PWM_TURN_ON 30
#define PWM_MAX 255
volatile int desiredTemperature = 35;
volatile float currentSpeed = 30;

// Data transmission
volatile unsigned char remainingOutData = 0;
volatile unsigned char outData[8];

void configurePWM() {
    // P1.2 is the output
    P1DIR |= BIT2; // Output
    P1SEL |= BIT2; // TA
    P1OUT &= ~BIT2; // Turn off

    // Setup the timer to be 1 MHz
    TA0CTL = TASSEL_2 | MC_1 | TACLR; // SMCLK, up, clear

    // Setup CCR0 for shutting down the LED
    TA0CCR0 = 255;

    // Start low
    TA0CCR1 = currentSpeed;
    TA0CCTL1 = OUTMOD_7;
}

void configureUARTLED() {
    // Onboard LED for UART debugging
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;
}

void configureUART() {
    P4SEL |= BIT5 | BIT4;
    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL_1;
    UCA1BR0 = 3; // 9600
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_3 | UCBRF_0;
    UCA1CTL1 &= ~UCSWRST;
    UCA1IE |= UCTXIE;
    UCA1IE |= UCRXIE;
}

void configureADC() {
    P6DIR &= ~BIT0;
    P6SEL |= BIT0;

    ADC12CTL2 = ADC12RES_2;
    ADC12CTL1 = ADC12SHP;
    ADC12CTL0 = ADC12SHT1_15 | ADC12SHT0_15 | ADC12MSC | ADC12ON | ADC12TOVIE | ADC12ENC | ADC12SC;
    ADC12IE = ADC12IE0; // Enable interrupt
    ADC12IFG &= ~ADC12IFG0; // Clear flag
}

int main(void)
{
    UCSCTL4 = SELA_0; // Enable ACLK (32.768 kHz signal)
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    configurePWM();
    configureUARTLED();
    configureUART();
    configureADC();

    __bis_SR_register(GIE);
    while (1);
}

#pragma vector=USCI_A1_VECTOR
__interrupt void uart(void) {
    P4OUT |= BIT7; // Turn on the onboard LED

    if (UCA1IFG & UCTXIFG) {
        UCA1IFG &= ~UCTXIFG; // Clear the TX flag
        if (remainingOutData > 0) {
            remainingOutData--;
            UCA1TXBUF = outData[remainingOutData];
        } else {
            __delay_cycles(5000);
            ADC12CTL0 |= ADC12SC;
        }
    }

    if (UCA1IFG & UCRXIFG) {
        unsigned char data = UCA1RXBUF; // Read data and clear flag
        desiredTemperature = data; // Set the new temperature to target
    }

    P4OUT &= ~BIT7; // Turn off the onboard LED
}

#pragma vector=ADC12_VECTOR
__interrupt void newADC(void) {
    switch (ADC12IV) {
    case 6: {
        // Read the A0 of the ADC
        unsigned int adcReading = ADC12MEM0;

        // Convert the value to Ohms
        float ohms = (adcReading * CONNECTED_RESISTOR) / (ADC_MAX - adcReading);

        // Convert the value to Kelvin
        float lnDiv = log(ohms / RREF);
        float tempK = 1.0 / (A1 + B1 * lnDiv + C1 * lnDiv * lnDiv + D1 * lnDiv * lnDiv * lnDiv);

        // Convert to Celcius
        float tempC = (tempK - 273.0);

        // Determine the offset and adjust the fan speed
        float difference = tempC - desiredTemperature;
        currentSpeed = currentSpeed + (difference * K);
        if (currentSpeed > 255) {
            currentSpeed = 255;
        }
        if (currentSpeed < 0) {
            currentSpeed = 0;
        }
        if (TA0CCR1 == PWM_OFF) {
            if (currentSpeed > PWM_OFF) {
                TA0CCR1 = PWM_TURN_ON;
            }
        } else {
            if (currentSpeed < PWM_MIN) {
                TA0CCR1 = PWM_OFF;
            } else if (currentSpeed > PWM_MAX) {
                TA0CCR1 = PWM_MAX;
            } else {
                TA0CCR1 = currentSpeed;
            }
        }

        remainingOutData = 7;
        int toSend = tempC * 100; // Two decimal places
        int i;
        for (i = 3; i < 8; i++) {
            if (i == 5) {
                outData[5] = '.';
            } else {
                outData[i] = (toSend % 10) + '0';
                toSend = toSend / 10;
            }
        }
        outData[2] = ' ';
        outData[1] = 'C';
        outData[0] = '\n';
        UCA1TXBUF = outData[7];
        break;
    }
    default:
        // Do nothing
    }
}
