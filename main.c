#include <msp432p401r.h>
#include <stdint.h>
#include "clock.h"

// Base Robot RSLK : PWM et détecteurs de piste
//=============================================
// Pierre-Yves Rochat, 2019.05.15, EPFL, pyr@pyr.ch
// Modifié par André Gomes, 2019.05.21, andre.gomes@epfl.ch

// LED du Launchpad MSP432 :
#define LedRougeInit P1DIR|=(1<<0);P1OUT&=~(1<<0)
#define LedRougeOn P1OUT|=(1<<0)
#define LedRougeOff P1OUT&=~(1<<0)
#define LedRougeToggle P1OUT^=(1<<0)

#define LedCoulInit P2DIR|=(1<<0)|(1<<1)|(1<<2);P2OUT&=~((1<<0)|(1<<1)|(1<<2))
#define LedCoulRougeOn P2OUT|=(1<<0)
#define LedCoulRougeOff P2OUT&=~(1<<0)
#define LedCoulRougeToggle P2OUT^=(1<<0)
#define LedCoulVertOn P2OUT|=(1<<1)
#define LedCoulVertOff P2OUT&=~(1<<1)
#define LedCoulVertToggle P2OUT^=(1<<1)
#define LedCoulBleuOn P2OUT|=(1<<2)
#define LedCoulBleuOff P2OUT&=~(1<<2)
#define LedCoulBleuToggle P2OUT^=(1<<2)

// Moteurs du robot :
#define MoteurDroiteInit P6DIR|=(1<<4);P6OUT&=~(1<<4);P6DIR|=(1<<5);P6OUT&=~(1<5)
#define MoteurDroiteOn P6OUT|=(1<<4)
#define MoteurDroiteOff P6OUT&=~(1<<4)
#define MoteurDroiteRecule P6OUT|=(1<<5)
#define MoteurDroiteAvance P6OUT&=~(1<<5)

#define MoteurGaucheInit P4DIR|=(1<<6);P4OUT&=~(1<<6);P1DIR|=(1<<5);P1OUT&=~(1<<5)
#define MoteurGaucheOn P4OUT|=(1<<6)
#define MoteurGaucheOff P4OUT&=~(1<<6)
#define MoteurGaucheRecule P1OUT|=(1<<5)
#define MoteurGaucheAvance P1OUT&=~(1<<5)

#define DetIrOn P3DIR|=(1<<7);P3OUT|=(1<<7)

// Variables globales :
volatile uint16_t PwmLedRouge; // géré par TA0 CCR1
volatile uint16_t PwmCoulRouge; // géré par TA0 CCR2
volatile uint16_t PwmCoulVert; // géré par TA0 CCR3
volatile uint16_t PwmCoulBleu; // géré par TA0 CCR4

volatile int16_t CommandeDroite; // géré par TA1 CCR1
volatile int16_t CommandeGauche; // géré par TA1 CCR2

volatile int16_t ValDetecteurs[8];
int16_t LimiteDetecteurs[] = {12000, 10000, 10000, 10000, 10000, 10000, 10000, 12000};
uint8_t BitDetecteurs[8];
uint8_t SommeBitDetecteurs;
uint8_t compteurCroisement=0;
uint8_t i;

volatile uint8_t FinCycle;

#define PeriodeCycle 50000
#define LimiteMoteur 100
#define LimiteLed 100

// Communication avec l'afficheur :
uint16_t idxDet;

void InitSerie() { // initialise la ligne série
    P3SEL0 |= BIT2 | BIT3; // UART2 pin en fonction secondaire P3.2 et P3.3
    UCA2CTLW0 |= UCSWRST; // Reset de l'automate de la liaison série
    UCA2CTLW0 |= UCSSEL__SMCLK; // sélection de l'horloge
    UCA2BR0 = (uint8_t)((uint32_t)48000000/(16*4*9600))%256; // choisit le baud-rate
    UCA2BR1 = (uint8_t)((uint32_t)48000000/(16*4*9600))/256;
    UCA2MCTLW = 0x1000 | UCOS16 | 0x0020;
    UCA2CTLW0 &= ~UCSWRST; // Initialise l'automate de la liaison série

    idxDet = 0;
}

char car;

void AfficheDetecteurs() { // envoie successivement les valeurs des détecteurs
    if (UCA2IFG & UCTXIFG) { // prêt à l'envoi
        UCA2TXBUF = ((((7-idxDet) & 0b1111)+8)<<4) | (ValDetecteurs[idxDet]>>11);
        idxDet++; if (idxDet>=8) idxDet = 0;
    }
}

int32_t Centre;

void CalculeSomme() {
    SommeBitDetecteurs = 0;
    for (i=0; i<8; i++) {
        SommeBitDetecteurs = SommeBitDetecteurs+BitDetecteurs[i];
    }
}

void CalculeCentre() {
    Centre = 0;
    Centre += ValDetecteurs[0]*7;
    Centre += ValDetecteurs[1]*5;
    Centre += ValDetecteurs[2]*3;
    Centre += ValDetecteurs[3]*1;
    Centre -= ValDetecteurs[4]*1;
    Centre -= ValDetecteurs[5]*3;
    Centre -= ValDetecteurs[6]*5;
    Centre -= ValDetecteurs[7]*7;
}

void AfficheCentre() {
    PwmLedRouge = 0; PwmCoulBleu = 0;
    if (Centre < 0) { PwmLedRouge = -Centre/(1+3+5+7); }
    if (Centre > 0) { PwmCoulBleu = Centre/(1+3+5+7); }
}

uint8_t Poussoir() {
    if (UCA2IFG & UCRXIFG) {
        car = UCA2RXBUF;
        if (car=='P') return 1;
            }
        return 0;
}

#define FACTEUR_P 20

// Programme principal :
//======================

void main() {
    WDTCTL = WDTPW | WDTHOLD;
    Clock_Init48MHz();
    P3SEL0 = 0; P3SEL1 = 0;

    LedRougeInit; LedCoulInit;
    PwmLedRouge;
    PwmCoulRouge = PwmCoulVert = PwmCoulBleu = 0;

    MoteurGaucheInit; MoteurDroiteInit;
    CommandeDroite = CommandeGauche = 0;

    InitSerie();

    FinCycle = 0;
    DetIrOn; // allume l'IR des détecteurs

    // Initialisation des timers :
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    TA0CTL = TASSEL_2 | MC_2 | ID_1 ;  // SMCLK, mode continu, DIV 2
    TA0CCTL0 = CCIE; // interruption TA0CCR0
    TA0CCR0 = PeriodeCycle; // période du PWM et des mesures
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);
    TA0CCTL1 = CCIE; // interruption TA0 CCR1
    TA0CCR1 = 0;
    TA0CCTL2 = CCIE; TA0CCR2 = 0;
    TA0CCTL3 = CCIE; TA0CCR3 = 0;
    TA0CCTL4 = CCIE; TA0CCR4 = 0;

    NVIC->ISER[0] = 1 << ((TA1_0_IRQn) & 31);
    TA1CTL = TASSEL_2 | MC_2 | ID_1 ;
    TA1CCTL0 = CCIE;
    TA1CCR0 = PeriodeCycle;
    NVIC->ISER[0] = 1 << ((TA1_N_IRQn) & 31);
    TA1CCTL1 = CCIE; TA1CCR1 = 0;
    TA1CCTL2 = CCIE; TA1CCR2 = 0;

    // Initialisation des ports :
    NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);
    P2IES |= (1<<3); P2IE |= (1<<3); P2IFG &=~(1<<3); // Détecteur 2  sur P2.3
    P2IES |= (1<<4); P2IE |= (1<<4); P2IFG &=~(1<<4); // Détecteur 6  sur P2.4
    P2IES |= (1<<6); P2IE |= (1<<6); P2IFG &=~(1<<6); // Détecteur 7  sur P2.7

    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);
    P3IES |= (1<<5); P3IE |= (1<<5); P3IFG &=~(61<<5); // Détecteur 0  sur P3.5

    NVIC->ISER[1] = 1 << ((PORT5_IRQn) & 31);
    P5IES |= (1<<1); P5IE |= (1<<1); P5IFG &=~(1<<1); // Détecteur 1  sur P5.1
    P5IES |= (1<<6); P5IE |= (1<<6); P5IFG &=~(1<<6); // Détecteur 5  sur P5.6

    NVIC->ISER[1] = 1 << ((PORT6_IRQn) & 31);
    P6IES |= (1<<6); P6IE |= (1<<6); P6IFG &=~(1<<6); // Détecteur 4  sur P6.6
    P6IES |= (1<<7); P6IE |= (1<<7); P6IFG &=~(1<<7); // Détecteur 3  sur P6.7

    __enable_interrupt();

    enum etats_machine{
        start = 0,
        pisteur = 1,
        init_croisement = 2,
        croisement = 3,
        arret = 4,
    };
    int etat = 0;

    while(1) {
        if (FinCycle) {
            for (i=0; i<8; i++) {
                if ((uint16_t)ValDetecteurs[i]> LimiteDetecteurs[i]) BitDetecteurs[i]=1; else BitDetecteurs[i]=0;
            }

            AfficheDetecteurs();
            CalculeCentre(); AfficheCentre(); CalculeSomme();

            switch(etat) {
                case start :
                    if (SommeBitDetecteurs>=7) {
                        etat = init_croisement; break;
                    } else {
                        etat = pisteur; 
                    }
	            break;

                case pisteur :
                    CommandeDroite = 15000-(Centre/FACTEUR_P); CommandeGauche = 15000+(Centre/FACTEUR_P); // Nouvelle valeur prises en compte dès le prochain cycle (dans l'interruption)
                    if (SommeBitDetecteurs>=7) {
                        etat = init_croisement; break;
                    }
                    break;

                case init_croisement :
                    compteurCroisement++;
                    if (compteurCroisement==4) {
                        etat = arret; break;
                    } else {
                        etat = croisement;
                    }
                    break;

                case croisement :
                    CommandeDroite = 15000; CommandeGauche = 15000;
                    if (SommeBitDetecteurs<7) {
                        etat = pisteur; break;
                    }
                    break;

                case arret :
                    CommandeDroite = 0; CommandeGauche = 0;
                    break;
            }
            FinCycle = 0;
        }
    }
}

// Routines d'interruptions :
//===========================

void PORT2_IRQHandler() {
    if (P2IFG & (1<<3)) { P2IFG &=~(1<<3); ValDetecteurs[2] = TA0R/2; } // IFG: si fanion vrai, retour fanion à 0 et enregistre valeur du timer
    if (P2IFG & (1<<4)) { P2IFG &=~(1<<4); ValDetecteurs[6] = TA0R/2; }
    if (P2IFG & (1<<6)) { P2IFG &=~(1<<6); ValDetecteurs[7] = TA0R/2; }
}

void PORT3_IRQHandler() {
    if (P3IFG & (1<<5)) { P3IFG &=~(1<<5); ValDetecteurs[0] = TA0R/2; }
}

void PORT5_IRQHandler() {
    if (P5IFG & (1<<1)) { P5IFG &=~(1<<1); ValDetecteurs[1] = TA0R/2; }
    if (P5IFG & (1<<6)) { P5IFG &=~(1<<6); ValDetecteurs[5] = TA0R/2; }
}

void PORT6_IRQHandler() {
    if (P6IFG & (1<<6)) { P6IFG &=~(1<<6); ValDetecteurs[4] = TA0R/2; }
    if (P6IFG & (1<<7)) { P6IFG &=~(1<<7); ValDetecteurs[3] = TA0R/2; }
}

// Timer A0 : PWM des LED et début des cycles des capteurs
void TA0_0_IRQHandler(void){ // TA0 CCR0 : fin (et début) des cycles
    volatile uint8_t i;

    P3DIR |= (1<<5); P3OUT |= (1<<5); // charge le condensateur
    P5DIR |= (1<<1); P5OUT |= (1<<1);
    P2DIR |= (1<<3); P2OUT |= (1<<3);
    P6DIR |= (1<<7); P6OUT |= (1<<7);
    P6DIR |= (1<<6); P6OUT |= (1<<6);
    P5DIR |= (1<<6); P5OUT |= (1<<6);
    P2DIR |= (1<<4); P2OUT |= (1<<4);
    P2DIR |= (1<<6); P2OUT |= (1<<6);

    TA0CCTL0 &=~ CCIFG; TA0R = 0; // début du cycle suivant

    if (PwmLedRouge>LimiteLed) LedRougeOn; // CCR1
    if (PwmCoulRouge>LimiteLed) LedCoulRougeOn; //CCR2
    if (PwmCoulVert>LimiteLed) LedCoulVertOn; //CCR3
    if (PwmCoulBleu>LimiteLed) LedCoulBleuOn; // CCR4

    TA0CCR1 = PwmLedRouge;
    TA0CCR2 = PwmCoulRouge;
    TA0CCR3 = PwmCoulVert;
    TA0CCR4 = PwmCoulBleu;

    FinCycle = 1;

    for (i=0; i<20; i++){} // charge des condensateurs

    P3DIR &=~(1<<5); // détecteur en entrée
    P5DIR &=~(1<<1);
    P2DIR &=~(1<<3);
    P6DIR &=~(1<<7);
    P6DIR &=~(1<<6);
    P5DIR &=~(1<<6);
    P2DIR &=~(1<<4);
    P2DIR &=~(1<<6);
}

void TA0_N_IRQHandler(void){
    TA0CCTL0 &=~ CCIFG;

    switch (TA0IV) {
        case 2 : LedRougeOff; break; // TA0 CCR1
        case 4 : LedCoulRougeOff; break; // TA0 CCR2
        case 6 : LedCoulVertOff; break; // TA0 CCR3
        case 8 : LedCoulBleuOff; break; // TA0 CCR4
        default : break;
    }
}

// Timer A1 : PWM des moteurs
void TA1_0_IRQHandler(void){ // TA1CCR0 : début des cycles PWM
    TA1CCTL0 &=~CCIFG; TA1R = 0; // début du cycle suivant

    MoteurDroiteOff; MoteurGaucheOff;
    if (CommandeDroite>LimiteMoteur) { TA1CCR1 = CommandeDroite*2; MoteurDroiteAvance; MoteurDroiteOn; }
    if (CommandeDroite<-LimiteMoteur) {  TA1CCR1 = -CommandeDroite*2; MoteurDroiteRecule; MoteurDroiteOn; }
    if (CommandeGauche>LimiteMoteur) {  TA1CCR2 = CommandeGauche*2; MoteurGaucheAvance; MoteurGaucheOn; }
if (CommandeGauche<-LimiteMoteur) { TA1CCR2 = -CommandeGauche*2;MoteurGaucheRecule; MoteurGaucheOn; }
}

void TA1_N_IRQHandler(void){
    TA1CCTL0 &=~CCIFG;

    switch (TA1IV) {
        case 2 : MoteurDroiteOff; break; // TA1 CCR1
        case 4 : MoteurGaucheOff; break; // TA1 CCR2
        default : break;
    }
}