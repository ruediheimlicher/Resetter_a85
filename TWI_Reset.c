//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>


// **********************************************************************
//Resetter							
//									
// **********************************************************************


// **********************************************************************
// **********************************************************************
#define SDA_LO_MAX            80 // counter, bis Reset infolge SDA_LO gesetzt wird (*0.25s)
#define SDA_HI_MAX            640 // counter, bis Reset infolge SDA_HI gesetzt wird
// **********************************************************************
// **********************************************************************
#define TEST 0

#define TWI_PORT		PORTB
#define TWI_PIN		PINB
#define TWI_DDR		DDRB


#define LOOPLEDPIN            0        // Blink-LED

#define SDAPIN                1        // Eingang von SDA/I2C
#define WEBSERVERPIN				2			// Eingang vom WebServer

#define OSZIPIN               3
#define OSZIHI                TWI_PORT |= (1<<OSZIPIN)
#define OSZILO                TWI_PORT &= ~(1<<OSZIPIN)
#define OSZITOGG              TWI_PORT ^= (1<<OSZIPIN)

#define REPORTPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO

#define RELAISPIN             4        // Schaltet Relais


//#define RESETCOUNT            0x200    // Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETDELAY            0x0F     // Waitcounter: Blockiert wiedereinschalten
#define WEBSERVERRESETCOUNT   0x08     // Counter bis Reset vom Webserver gesetzt wird

#define WAIT                  0
#define CHECK                 1 // in ISR gesetzt, resetcount soll erhoeht werden

#define SDA_LO_RESET          2 // gesetzt, wenn SDA zulange LO ist
#define SDA_HI_RESET          3 // gesetzt, wenn SDA zulange HI ist
#define WEBSERVER_RESET       4 // Reset soll starten



volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	webserverresetcount=0; // Zeit, die der Resetrequest  vom Webserver dauert
volatile uint16_t   webserverresetdelaycount=0; // delay des Relais
volatile uint8_t statusflag=0;

volatile uint16_t	overflowcount=0;

volatile uint16_t	SDA_LO_counter=0;
volatile uint16_t	SDA_HI_counter=0;

volatile char ii=0;

volatile uint16_t   wdtcounter=0;
void slaveinit(void)
{
   
   CLKPR |= (1<<3);
   TWI_DDR |= (1<<LOOPLEDPIN);
   
   TWI_DDR |= (1<<RELAISPIN);       // Ausgang: Schaltet Reset-Relais fuer Zeit RESETDELAY
   TWI_PORT &= ~(1<<RELAISPIN);     // LO   
   
   TWI_DDR |= (1<<OSZIPIN);        // Ausgang
   TWI_PORT |= (1<<OSZIPIN);       // HI

   
   TWI_DDR &= ~(1<<SDAPIN);        // Eingang: Verbunden mit SDA, misst LO-Zeit, um Stillstand zu erkennen
   TWI_PORT |= (1<<SDAPIN);        // HI
   
   
   TWI_DDR &= ~(1<<WEBSERVERPIN);        // Eingang: Verbunden mit Webserver, empfŠngt LO-Signal zum reset
   TWI_PORT |= (1<<WEBSERVERPIN);        // HI
   
   
   MCUCR |= 1<<ISC00; // Pin  change
   GIMSK |= 1<<PCIE;
   PCMSK |= 1<<PCINT1;
   
   
   //TWI_DDR &= ~(1<<VCCPIN);	// Eingang, Abfragen von VCC von Master
   //TWI_PORT |= (1<<VCCPIN);	// HI
   
   //TWI_DDR &= ~(1<<SCLPIN);	// Eingang
   //TWI_PORT |= (1<<SCLPIN);	// HI
   
}




void WDT_Init(void)
{
   cli();
   MCUSR &= ~(1<<WDRF);
   WDTCR = (1<<WDCE) | (1<<WDE) ;
  
   wdt_enable(WDTO_2S);
   WDTCR |= (1<< WDIE); // muss nach wdt_enable stehen
   sei();
}

ISR(WDT_vect)
{
   PORTB |= (1<<RELAISPIN);
 }


/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler */
	TCCR0B = (1 << CS00)|(1 << CS02); // clock/1024
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK = (1 << TOIE0); // TOV0 Overflow
}

ISR(TIM0_OVF_vect) // ca. 0.25s
{
   statusflag |= (1<<CHECK);
//   TWI_PORT ^=(1<<LOOPLEDPIN);
   
}

ISR(PCINT0_vect) // Potential-Aenderung von SDA
{
   if ((!(statusflag & (1<<WAIT))))// WAIT verhindert, dass Relais von SDA_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      SDA_HI_counter=0;
      SDA_LO_counter=0;
      webserverresetcount=0;      
   }   
}

int main (void) 
{
   
   wdt_disable();
   MCUSR = 0; // clear reset flags
   //MCUSR &= ~(1<<WDRF);// Just to be safe since we can not clear WDE if WDRF is set
   slaveinit();
   cli();
   
   //WDTCR &= ~(1<<WDE);
  // WDTCR |= (1<<WDCE) | (1<<WDE);
   //wdt_enable(WDTO_1S);
   //WDTCR |=  (1<< WDIE); // Watchdog macht nur reset
   // MCUSR &= ~(1<<WDRF); // clear the watchdog reset
    WDT_Init();
   
   
   //MCUCR |= (1<<ISC00);
   //   GIMSK |= (1<<INT0);
   timer_init();
   sei();
   uint8_t i=0;
   for(i=0;i<3;i++)
   {
      
      TWI_PORT &= ~(1<<REPORTPIN);
      _delay_ms(100);
      TWI_PORT |= (1<<REPORTPIN);
      _delay_ms(100);
   }
 
   
#pragma mark while
   while (1)
   {
      //
      //wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=0x00AF)
      {
         loopcount0=0;
         
         loopcount1++;
         if (loopcount1 >0x8F)
         {
           TWI_PORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
         }
         
      }
      if (TEST)
      {
         if (TWI_PIN & (1<<WEBSERVERPIN) )
         {
            while (wdtcounter < 8) // Zeit verspielen -> wdt ausloesen
            {
               //PORTB ^= (1<<RELAISPIN);
               wdtcounter++;
               //TWI_PORT &=~(1<<OSZIPIN);
               _delay_ms(100);
            }
            //TWI_PORT |=(1<<OSZIPIN);
            //wdt_reset();
            //else
            {
               //TWI_PORT &=~(1<<RELAISPIN);
               wdtcounter=0;
            }
            
            /*
             if (ii>5)
             {
             TWI_PORT &=~(1<<RELAISPIN);
             ii=0;
             }
             */
            
         }
         else
         {
            wdt_reset();
            wdtcounter=0;
            TWI_PORT &=~(1<<RELAISPIN);
         }
      }
      else 
      {
         wdt_reset();
      }
      
       /*
       Checken, ob SDA zu lange auf gleichem Wert blieb oder ein Reset vom Webserver verlangt wird
       */
      
      
      if (statusflag & (1<<CHECK))// Timer gibt Takt der Abfrage an, 0.5s
      {
         // Reset durch Webserver: WEBSERVERPIN abfragen: Reset wenn LO
         
         if ((TWI_PIN & (1 << WEBSERVERPIN))) // all OK
         {
 
         }
         else // Reset durch Webserver starten
         {   
            if (!(statusflag & (1<<WAIT)))
            {
               if (webserverresetcount < WEBSERVERRESETCOUNT) // noch warten
               {
                  webserverresetcount++;
               }
               else
               {
                  statusflag |= (1<<WEBSERVER_RESET);
                  statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
                  delaycount = 0; // Dauer des Relais-Impulses
                  webserverresetcount=0;
               }
            }//  not WAIT
         }// Reset durch Webserver starten
         
      
      
         // impulsdauer von SDA checken
         if (PINB & (1<<SDAPIN)) // HI, darf nicht zu lange dauern, 3 min
         {
            if (!(statusflag & (1<<WAIT)))
            {
               if (SDA_HI_counter < SDA_HI_MAX) // noch abwarten
               {
                  SDA_HI_counter++;
               }
               else
               {
                  statusflag |= (1<<SDA_HI_RESET); // Reset aktivieren
                  statusflag |= (1<<WAIT);// Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
                  delaycount = 0; 
                  SDA_HI_counter=0;
               }
            }
         }
         else // LO, sollte nicht zu lange dauern, 2s
         {
            if (!(statusflag & (1<<WAIT)))
            {
               if (SDA_LO_counter < SDA_LO_MAX) // noch abwarten
               {
                  SDA_LO_counter++;
               }
               else
               {
                  statusflag |= (1<<SDA_LO_RESET); // Reset aktivieren
                  statusflag |= (1<<WAIT); // Warten auf Ende Reset, Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
                  delaycount = 0; 
                  SDA_LO_counter=0;
               }
            }
         } // SDA LO
         // resetcount wird bei Aenderungen am SDA  in ISR von INT0 zurueckgesetzt. (Normalbetrieb)
         
         // statusflag abarbeiten

         if ((statusflag & (1<<SDA_HI_RESET)) || (statusflag & (1<<SDA_LO_RESET)) || (statusflag & (1<<WEBSERVER_RESET))) // Reset starten/halten
         {
            if (TWI_PIN & (1<<RELAISPIN)) // Relais ist schon ON, zaehlen
            {
               delaycount++;
               if (delaycount > RESETDELAY) // 15 // Ende reset
               {
                  TWI_PORT &= ~(1<<RELAISPIN);
                  statusflag &= ~0x1C ; // alle reset-Bits (2,3,4)
                  statusflag &= ~(1<<WAIT); 
                  TWI_PORT |= (1<<REPORTPIN); // Report an Webserver beenden
               }
            }
            else
            {
               TWI_PORT |= (1<<RELAISPIN); // Relais ON
               delaycount = 0;
               TWI_PORT &= ~(1<<REPORTPIN); // Report an Webserver, active LO
            }
            // RELAISPIN Hi, Relais schaltet Bus aus
            // WAIT ist gesetzt, Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
               
            
         }
       
         statusflag &= ~(1<<CHECK);
      } // if check
       
   }//while
   return 0;
}
