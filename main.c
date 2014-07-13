#include <xc.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
#pragma config IESO = ON               // Two-speed Oscillator Start-Up Enable (Start up with user-selected oscillator)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = ON              // JTAG Port Enable (JTAG is Enabled)

#define    FCY    40000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include <libpic30.h>        // __delayXXX() functions macros defined here
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define delay_us    __delay_us
#define delay_ms    __delay_ms

//////////////////////
short clean = 0;
short cha = 'Q'; //Dato Recibido por Uart
char CN = 0;
short cant = 0;
short cont = 0;
char itm = 0;
char ti = 0;
char deb = 0;
short nm = 250;
int nmu = 250;
short const_d = 47;
short const_r = 3200;
double const_div = 47.00;
double const_res = 3200.00;
double const_mult = 10000.00;
char fail[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
double acumulador[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

struct sen {
    unsigned sub : 1;
    uint16_t tinit : 16;
    uint16_t th : 16;
    uint16_t tt : 16;
}
s1 = {0, 0, 0, 0},
s2 = {0, 0, 0, 0},
s3 = {0, 0, 0, 0},
s4 = {0, 0, 0, 0},
s5 = {0, 0, 0, 0},
s6 = {0, 0, 0, 0},
s7 = {0, 0, 0, 0},
s8 = {0, 0, 0, 0};

void putch(char val) {
    while (!U1STAbits.UTXBF);
    U1TXREG = val;
}

void formul(char ns, short ti, short th, short tt) {
    double dc = 0;
    double temp = 0;
    if (th != 0 && tt != 0 && th < tt) {
        dc = (double) th / tt;
        dc = (double)dc * const_mult;
        temp = (double) (dc - const_r);
        temp = (double) temp / const_d;
        printf(";%u;%3.4f&\r\n", ns, temp);
    } else
        printf(";%u;Err&\r\n", ns);
    if (deb == 1) {
        printf("\r\ndc=%3.6f", dc);
        printf("\r\nti=%u", ti);
        printf("\r\nth=%u", th);
        printf("\r\ntt=%u", tt);
    }
}

void formulx(short ns, short th, short tt) {
    if (th != 0 && tt != 0 && th < tt) {
        double dc = (double) th / tt;
        acumulador[ns] += dc;
    } else {
        fail[ns]++;
    }
}

void dis() {
    IEC0bits.IC1IE = 0; // inhabilitar interrupcion de Captura1
    IEC0bits.IC2IE = 0; // inhabilitar interrupcion de Captura2
    IEC2bits.IC3IE = 0; // inhabilitar interrupcion de Captura3
    IEC2bits.IC4IE = 0; // inhabilitar interrupcion de Captura4
    IEC2bits.IC5IE = 0; // inhabilitar interrupcion de Captura5
    IEC2bits.IC6IE = 0; // inhabilitar interrupcion de Captura6
    IEC1bits.IC7IE = 0; // inhabilitar interrupcion de Captura7
    IEC1bits.IC8IE = 0; // inhabilitar interrupcion de Captura8
    IC1CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC2CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC3CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC4CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC5CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC6CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC7CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC8CONbits.ICM = 0b000; //Capture on every Edge (R/F)
}

void limpiar() {
    s1.sub = 0;
    s1.th = 0;
    s1.tt = 0;
    s1.tinit = 0;
    s2.sub = 0;
    s2.th = 0;
    s2.tt = 0;
    s2.tinit = 0;
    s3.sub = 0;
    s3.th = 0;
    s3.tt = 0;
    s3.tinit = 0;
    s4.sub = 0;
    s4.th = 0;
    s4.tt = 0;
    s4.tinit = 0;
    s5.sub = 0;
    s5.th = 0;
    s5.tt = 0;
    s5.tinit = 0;
    s6.sub = 0;
    s6.th = 0;
    s6.tt = 0;
    s6.tinit = 0;
    s7.sub = 0;
    s7.th = 0;
    s7.tt = 0;
    s7.tinit = 0;
    s8.sub = 0;
    s8.th = 0;
    s8.tt = 0;
    s8.tinit = 0;
    while (IC1CONbits.ICBNE) clean = IC1BUF;
    while (IC2CONbits.ICBNE) clean = IC2BUF;
    while (IC3CONbits.ICBNE) clean = IC3BUF;
    while (IC4CONbits.ICBNE) clean = IC4BUF;
    while (IC5CONbits.ICBNE) clean = IC1BUF;
    while (IC6CONbits.ICBNE) clean = IC2BUF;
    while (IC7CONbits.ICBNE) clean = IC3BUF;
    while (IC8CONbits.ICBNE) clean = IC4BUF;
}

void en() {
    IEC0bits.IC1IE = 1; // inhabilitar interrupcion de Captura1
    IEC0bits.IC2IE = 1; // inhabilitar interrupcion de Captura2
    IEC2bits.IC3IE = 1; // inhabilitar interrupcion de Captura3
    IEC2bits.IC4IE = 1; // inhabilitar interrupcion de Captura4
    IEC2bits.IC5IE = 1; // inhabilitar interrupcion de Captura5
    IEC2bits.IC6IE = 1; // inhabilitar interrupcion de Captura6
    IEC1bits.IC7IE = 1; // inhabilitar interrupcion de Captura7
    IEC1bits.IC8IE = 1; // inhabilitar interrupcion de Captura8
    limpiar();
    IC1CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC2CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC3CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC4CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC5CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC6CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC7CONbits.ICM = 0b001; //Capture on every Edge (R/F)
    IC8CONbits.ICM = 0b001; //Capture on every Edge (R/F)
}

void tomdat() {
    TMR2 = 0;
    en();
    delay_ms(1);
    delay_us(500);
    s1.tinit = IC1BUF;
    s1.th = IC1BUF - s1.tinit;
    s1.tt = IC1BUF - s1.tinit;
    s2.tinit = IC2BUF;
    s2.th = IC2BUF - s2.tinit;
    s2.tt = IC2BUF - s2.tinit;
    s3.tinit = IC3BUF;
    s3.th = IC3BUF - s3.tinit;
    s3.tt = IC3BUF - s3.tinit;
    s4.tinit = IC4BUF;
    s4.th = IC4BUF - s4.tinit;
    s4.tt = IC4BUF - s4.tinit;
    s5.tinit = IC5BUF;
    s5.th = IC5BUF - s5.tinit;
    s5.tt = IC5BUF - s5.tinit;
    s6.tinit = IC6BUF;
    s6.th = IC6BUF - s6.tinit;
    s6.tt = IC6BUF - s6.tinit;
    s7.tinit = IC7BUF;
    s7.th = IC7BUF - s7.tinit;
    s7.tt = IC7BUF - s7.tinit;
    s8.tinit = IC8BUF;
    s8.th = IC8BUF - s8.tinit;
    s8.tt = IC8BUF - s8.tinit;
    dis();
    formulx(1, s1.th, s1.tt);
    formulx(2, s2.th, s2.tt);
    formulx(3, s3.th, s3.tt);
    formulx(4, s4.th, s4.tt);
    formulx(5, s5.th, s5.tt);
    formulx(6, s6.th, s6.tt);
    formulx(7, s7.th, s7.tt);
    formulx(8, s8.th, s8.tt);
}

void tomdatp() {
    TMR2 = 0;
    en();
    delay_ms(1);
    delay_us(500);
    s1.tinit = IC1BUF;
    s1.th = IC1BUF - s1.tinit;
    s1.tt = IC1BUF - s1.tinit;
    s2.tinit = IC2BUF;
    s2.th = IC2BUF - s2.tinit;
    s2.tt = IC2BUF - s2.tinit;
    s3.tinit = IC3BUF;
    s3.th = IC3BUF - s3.tinit;
    s3.tt = IC3BUF - s3.tinit;
    s4.tinit = IC4BUF;
    s4.th = IC4BUF - s4.tinit;
    s4.tt = IC4BUF - s4.tinit;
    s5.tinit = IC5BUF;
    s5.th = IC5BUF - s5.tinit;
    s5.tt = IC5BUF - s5.tinit;
    s6.tinit = IC6BUF;
    s6.th = IC6BUF - s6.tinit;
    s6.tt = IC6BUF - s6.tinit;
    s7.tinit = IC7BUF;
    s7.th = IC7BUF - s7.tinit;
    s7.tt = IC7BUF - s7.tinit;
    s8.tinit = IC8BUF;
    s8.th = IC8BUF - s8.tinit;
    s8.tt = IC8BUF - s8.tinit;
    dis();
    formul(1, s1.tinit, s1.th, s1.tt);
    formul(2, s2.tinit, s2.th, s2.tt);
    formul(3, s3.tinit, s3.th, s3.tt);
    formul(4, s4.tinit, s4.th, s4.tt);
    formul(5, s5.tinit, s5.th, s5.tt);
    formul(6, s6.tinit, s6.th, s6.tt);
    formul(7, s7.tinit, s7.th, s7.tt);
    formul(8, s8.tinit, s8.th, s8.tt);
}

//void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
//    TERR = 1;
//    IEC0bits.T2IE = 0; //disable_interrupts(INT_TIMER2);
//    IFS0bits.T2IF = 0; // Clear Timer1 Interrupt Flag}
//}

//void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
//    concon++;
//    if (concon > 125) {
//        led = ~led;
//        concon = 0;
//    }
//    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag}
//}

void init_toma() {
    if (itm == 0) {
        const_div = (double) const_d * nmu;
        const_res = (double) const_r * nmu;
        acumulador[1] = 0;
        acumulador[2] = 0;
        acumulador[3] = 0;
        acumulador[4] = 0;
        acumulador[5] = 0;
        acumulador[6] = 0;
        acumulador[7] = 0;
        acumulador[8] = 0;
        fail[1] = 0;
        fail[2] = 0;
        fail[3] = 0;
        fail[4] = 0;
        fail[5] = 0;
        fail[6] = 0;
        fail[7] = 0;
        fail[8] = 0;
        cant = nmu; //cha;
        ti = 1;
        itm = 1;
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    cha = U1RXREG;
    if (CN == 0) {
        switch (cha) {
            case('A'):
            {
                //Modo rápido: 250 Muestras
                nmu = nm;
                init_toma();
                break;
            }
            case('B'):
            {
                //Modo número: Número de muestras*250
                CN = 1;
                break;
            }
        }
    } else {
        nmu = (int)cha * nm;
        init_toma();
    }
    IFS0bits.U1RXIF = 0; // Clear Timer1 Interrupt Flag}
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) {
    if (!s1.sub) {
        if (!PORTDbits.RD8)
            clean = IC1BUF;
        else
            s1.sub = 1;
    } else if (PORTDbits.RD8) {
        IEC0bits.IC1IE = 0;
        s1.sub = 0;
    }
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void) {
    if (!s2.sub) {
        if (!PORTDbits.RD9)
            clean = IC2BUF;
        else
            s2.sub = 1;
    } else if (PORTDbits.RD9) {
        IEC0bits.IC2IE = 0;
        s2.sub = 0;
    }
    IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC3Interrupt(void) {
    if (!s3.sub) {
        if (!PORTDbits.RD10)
            clean = IC3BUF;
        else
            s3.sub = 1;
    } else if (PORTDbits.RD10) {
        IEC2bits.IC3IE = 0;
        s3.sub = 0;
    }
    IFS2bits.IC3IF = 0; // Clear IC3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC4Interrupt(void) {
    if (!s4.sub) {
        if (!PORTDbits.RD11)
            clean = IC4BUF;
        else
            s4.sub = 1;
    } else if (PORTDbits.RD11) {
        IEC2bits.IC4IE = 0;
        s4.sub = 0;
    }
    IFS2bits.IC4IF = 0; // Clear IC4 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC5Interrupt(void) {
    if (!s5.sub) {
        if (!PORTDbits.RD4)
            clean = IC5BUF;
        else
            s5.sub = 1;
    } else if (PORTDbits.RD4) {
        IEC2bits.IC5IE = 0;
        s5.sub = 0;
    }
    IFS2bits.IC5IF = 0; // Clear IC5 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC6Interrupt(void) {
    if (!s6.sub) {
        if (!PORTDbits.RD5)
            clean = IC6BUF;
        else
            s6.sub = 1;
    } else if (PORTDbits.RD5) {
        IEC2bits.IC6IE = 0;
        s6.sub = 0;
    }
    IFS2bits.IC6IF = 0; // Clear IC6 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void) {
    if (!s7.sub) {
        if (!PORTBbits.RB4)
            clean = IC7BUF;
        else
            s7.sub = 1;
    } else if (PORTBbits.RB4) {
        IEC1bits.IC7IE = 0;
        s7.sub = 0;
    }
    IFS1bits.IC7IF = 0; // Clear IC7 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _IC8Interrupt(void) {
    if (!s8.sub) {
        if (!PORTBbits.RB5)
            clean = IC8BUF;
        else
            s8.sub = 1;
    } else if (PORTBbits.RB5) {
        IEC1bits.IC8IE = 0;
        s8.sub = 0;
    }
    IFS1bits.IC8IF = 0; // Clear IC8 Interrupt Flag
}

void report(short ns) {
    if (fail[ns] == 0) {
        double calc1 = (double) acumulador[ns] * const_mult;
        double calc2 = (double) calc1 - const_res;
        double temp = (double) calc2 / const_div;
        printf(";%u;%3.4f&\r\n", ns, temp);
    } else {
        printf(";%u;Err&\r\n", ns);
    }
}

int main(int argc, char** argv) {

    /////////Configuración de Clock/////////
    CLKDIVbits.ROI = 0;
    CLKDIVbits.DOZE = 0b000;
    CLKDIVbits.DOZEN = 0;
    CLKDIVbits.PLLPRE = 0b00010; //N1 = 4
    PLLFBDbits.PLLDIV = 0b000011110; //M = 32
    CLKDIVbits.PLLPOST = 0b00; //N2 = 2;
    //(20*32)/(2*4) = 5*16 = 80 MHz

    /////////Secuencia Cambio de Oscilador/////////
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b011);
    // Wait for PLL to lock
    while (!OSCCONbits.LOCK);
    //////////////////////////////////////////////

    //Inicialización de Variables//
    cha = 'Q';
    cant = 0;
    cont = 0;
    CN = 0;
    itm = 0;
    ti = 0;
    deb = 0;
    ///////////////////////////////
    AD1PCFGLbits.PCFG4 = 1;
    AD1PCFGLbits.PCFG5 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISFbits.TRISF2 = 1; //U1RX
    TRISGbits.TRISG2 = 0;
    TRISGbits.TRISG3 = 1;
    TRISFbits.TRISF6 = 0;
    LATGbits.LATG2 = 1;
    LATFbits.LATF6 = 0;
    if (PORTGbits.RG3)
        deb = 1;
    //    /////////Configuración timer 1/////////
    //    T1CONbits.TSIDL = 1;
    //    T1CONbits.TCS = 0;
    //    T1CONbits.TGATE = 0;
    //    T1CONbits.TCKPS = 0b00; //1:1 PreScaller 0.0625 us per ++
    //    PR1 = 65535; //periodo del timer 1
    //    IPC0bits.T1IP = 5; // Prioridad 1 para inttimer1
    //    IFS0bits.T1IF = 0; // limpiar flag de interrupcion 1
    //    IEC0bits.T1IE = 1; // habilitar interrupcion del timer1
    //    T1CONbits.TON = 1; // iniciar timer 1

    ///////Configuración timer 2/////////
    T2CONbits.TSIDL = 1;
    T2CONbits.TCS = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0b00; //1:8 PreScaller 0.5us per ++
    PR2 = 65535; //periodo del timer 2
    IPC1bits.T2IP = 5; // Prioridad 1 para inttimer2
    IFS0bits.T2IF = 0; // limpiar flag de interrupcion 2
    IEC0bits.T2IE = 0; // habilitar interrupcion del timer2
    T2CONbits.TON = 1; // iniciar timer 2

    ////////////configuración de ICx///////
    limpiar();
    IC1CONbits.ICSIDL = 1; //free-Running Mode  //0b10100; //IC1 Trig Souce  = IC1 pin
    IC2CONbits.ICSIDL = 1; //free-Running Mode  //0b10101; //IC2 Trig Souce  = IC2 pin
    IC3CONbits.ICSIDL = 1; //free-Running Mode  //0b10101; //IC2 Trig Souce  = IC2 pin
    IC4CONbits.ICSIDL = 1; //free-Running Mode  //0b10101; //IC2 Trig Souce  = IC2 pin
    IC5CONbits.ICSIDL = 1; //IC1 halts on idle
    IC6CONbits.ICSIDL = 1; //IC2 halts on idle
    IC7CONbits.ICSIDL = 1; //IC1 halts on idle
    IC8CONbits.ICSIDL = 1; //IC1 halts on idle
    IC1CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC2CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC3CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC4CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC5CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC6CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC7CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC8CONbits.ICTMR = 1; //IC1 Captute Timer = Timer2
    IC1CONbits.ICI = 0b00; //Interrupt on every capture event
    IC2CONbits.ICI = 0b00; //Interrupt on every capture event
    IC3CONbits.ICI = 0b00; //Interrupt on every capture event
    IC4CONbits.ICI = 0b00; //Interrupt on every capture event
    IC5CONbits.ICI = 0b00; //Interrupt on every capture event
    IC6CONbits.ICI = 0b00; //Interrupt on every capture event
    IC7CONbits.ICI = 0b00; //Interrupt on every capture event
    IC8CONbits.ICI = 0b00; //Interrupt on every capture event
    IC1CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC2CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC3CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC4CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC5CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC6CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC7CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    IC8CONbits.ICM = 0b000; //Capture on every Edge (R/F)
    TMR2 = 0;
    IPC0bits.IC1IP = 1; // Prioridad 1 para Captura1
    IPC1bits.IC2IP = 1; // Prioridad 1 para Captura2
    IPC9bits.IC3IP = 1; // Prioridad 1 para Captura3
    IPC9bits.IC4IP = 1; // Prioridad 1 para Captura4
    IPC9bits.IC5IP = 1; // Prioridad 1 para Captura5
    IPC10bits.IC6IP = 1; // Prioridad 1 para Captura6
    IPC5bits.IC7IP = 1; // Prioridad 1 para Captura7
    IPC5bits.IC8IP = 1; // Prioridad 1 para Captura8
    IFS0bits.IC1IF = 0; // limpiar flag de interrupcion IC1
    IFS0bits.IC2IF = 0; // limpiar flag de interrupcion IC2
    IFS2bits.IC3IF = 0; // limpiar flag de interrupcion IC3
    IFS2bits.IC4IF = 0; // limpiar flag de interrupcion IC4
    IFS2bits.IC5IF = 0; // limpiar flag de interrupcion IC5
    IFS2bits.IC6IF = 0; // limpiar flag de interrupcion IC6
    IFS1bits.IC7IF = 0; // limpiar flag de interrupcion IC7
    IFS1bits.IC8IF = 0; // limpiar flag de interrupcion IC8
    IEC0bits.IC1IE = 0; // inhabilitar interrupcion de Captura1
    IEC0bits.IC2IE = 0; // inhabilitar interrupcion de Captura2
    IEC2bits.IC3IE = 0; // inhabilitar interrupcion de Captura3
    IEC2bits.IC4IE = 0; // inhabilitar interrupcion de Captura4
    IEC2bits.IC5IE = 0; // inhabilitar interrupcion de Captura5
    IEC2bits.IC6IE = 0; // inhabilitar interrupcion de Captura6
    IEC1bits.IC7IE = 0; // inhabilitar interrupcion de Captura7
    IEC1bits.IC8IE = 0; // inhabilitar interrupcion de Captura8

    /////////Configuración UART 1/////////
    //    U1BRG = 259; //BaudRate = 9600;
    U1BRG = 64; //BaudRate = 38400;
    //    U1BRG = 21; //BaudRate = 115200;
    U1MODEbits.USIDL = 0; //1?
    U1MODEbits.IREN = 0;
    U1MODEbits.RTSMD = 1;
    U1MODEbits.UEN = 0b00; //11?
    U1MODEbits.WAKE = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0;
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXINV = 0;
    U1STAbits.UTXBRK = 0;
    U1STAbits.URXISEL = 0b01;
    U1STAbits.ADDEN = 0;
    U1STAbits.RIDLE = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    IPC2bits.U1RXIP = 2; // Prioridad 1 para inttimer1
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //    U1STAbits.UTXBF //recurso, buffer vacio
    //    U1STAbits.URXDA //recurso, datos recibidos

    //Protocolo Inicial
    delay_ms(1000);
    printf("LT8-WL\r\n");
    printf("Sensores:\r\n");
    tomdatp();
    printf("Funcionamiento Correcto\r\n");
    printf("Esperando Comandos.....\r\n");
    ///////////////////
    while (1) {
        if (ti == 1) {
            printf("Toma de %u Muestras Iniciada!#\r\n", cant);
            ti = 0;
        }
        cont = 0;
        while (itm == 1 && ti == 0) {
            tomdat();
            cont++;
            if (cont >= cant) {
                report(1);
                report(2);
                report(3);
                report(4);
                report(5);
                report(6);
                report(7);
                report(8);
                printf("#Toma de %u Muestras Terminada!\r\n", cant);
                printf("#%u#/", cant);
                cant = 0;
                itm = 0;
                CN = 0;
            }
        }
    }
    return 1;
}

