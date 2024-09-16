/*
 * project2.c
 *
 *  Created on: 16 Sep 2024
 *      Author: maria
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* prototypes */
void TIMER1_COMP_Init();
void INT0_Init();
void INT1_Init();
void INT2_Init();
void update_display();
void buttons_inc_dec();

/*struct to store counter values*/
struct display
{
    unsigned char hrs;
    unsigned char mins;
    unsigned char secs;
} time;

unsigned char digit_values[6]; // BCD values array

unsigned char current_disp, countdown_mode = 0, pause_flag = 0; //number of current display to show

ISR(TIMER1_COMPA_vect)
{
	if(countdown_mode==0)
	{
		/* count up mode*/
		if (time.secs == 59)
		{
			time.secs = 0;
			if (time.mins == 59)
			{
				time.mins = 0;
				if (time.hrs == 23)
				{
					time.hrs = 0;
				}
				else
				{
					time.hrs++;
				}
			}
			else
			{
				time.mins++;
			}
		}
		else
		{
			time.secs++;
		}
	}

	/*count down mode*/
	else
	{
	    if (time.secs == 0)
	    {
	    	if (time.mins == 0)
	    	{
	    		if (time.hrs==0)
	    		{
	    			//buzzer on
	    			PORTD |= (1<<PD0);
	    		}
	    		else
	    		{
	    			time.mins=59;
	    			time.hrs--;
	    		}
	    	}
	    	else
	    	{
	    		time.secs=59;
	    		time.mins--;
	    	}
	    }
	    else
	    {
	    	time.secs--;
	    }
	}
	update_display();
}

// reset
ISR(INT0_vect)
{
    time.hrs = 0;
    time.mins = 0;
    time.secs = 0;
	update_display();
    TCNT1 = 0;
}

// pause
ISR(INT1_vect)
{
    TCCR1B &= 0xF8; //clear prescaler to pause timer
    pause_flag=1;
}

//resume
ISR(INT2_vect)
{
    TCCR1B =(1<<CS12); //set prescaler to resume timer
    pause_flag=0;
}

void TIMER1_COMP_Init()
{
    TCNT1 = 0;       // initialize timer to zero
    OCR1A = 62499;   // set compares for every 1 second

    TIMSK |= (1<<OCIE1A); // enable interrupt

    TCCR1A = (1<<FOC1A) | (1<<FOC1B); // non-pwm mode

    TCCR1B = (1<<WGM12) | (1<<CS12); //CTC mode, prescaler 256
}

void INT0_Init()
{
    DDRD &= ~(1<<PD2);
    PORTD |= (1<<PD2);   //internal pull-up
    MCUCR |= (1<<ISC01); // trigger on falling edge
    GICR |= (1<<INT0);   // enable interrupt
}

void INT1_Init()
{
    DDRD &= ~(1<<PD3);   // Configure PD3 as input
    PORTD &= ~ (1<< PD3); // pull up disable
    MCUCR |= (1<<ISC11) | (1<<ISC10); // trigger on rising edge
    GICR |= (1<<INT1);   // enable interrupt
}

void INT2_Init()
{
    DDRB &= ~(1<<PB2);
    PORTB |= (1<<PB2); // internal pull up
    MCUCSR &= ~(1<<ISC2); // trigger on falling edge
    GICR |= (1<<INT2);   // enable interrupt
}

// convert time to BCD for each digit
void update_display()
{
    digit_values[0] = time.hrs / 10;    // hours tens
    digit_values[1] = time.hrs % 10;    // hours units
    digit_values[2] = time.mins / 10;   // mins tens
    digit_values[3] = time.mins % 10;   // mins units
    digit_values[4] = time.secs / 10;   // secs tens
    digit_values[5] = time.secs % 10;   // secs units
}

// multiplex the displays
void show_display()
{
    for (current_disp = 0; current_disp < 6; current_disp++)
    {
        PORTA = 0;                  // turn off all displays
        PORTA |= (1<<current_disp); // activate current display
        PORTC = (PORTC & 0xF0) | (digit_values[current_disp] & 0x0F); // set display value
        _delay_ms(5);               // delay
    }
}

void buttons_inc_dec()
{
	/* button increment and decrement*/
	if(!(PINB & (1<<PB1)))
	{
		if(time.hrs == 99)
		{
			time.hrs = 0;
		}
		else
		{
		(time.hrs)++;
		}

		update_display();

		while(!(PINB & (1<<PB1)))
		{show_display();}}

	if(!(PINB & (1<<PB0)))
	{
		if (time.hrs == 0)
		{
			time.hrs = 99;
		}
		else
		{
		(time.hrs)--;
		}

		update_display();

		while(!(PINB & (1<<PB0)))
		{show_display();}
	}

	if(!(PINB & (1<<PB4)))
	{
		if(time.mins == 59)
		{
			time.mins = 0;
		}
		else
		{
		(time.mins)++;
		}

		update_display();

		while(!(PINB & (1<<PB4)))
		{show_display();}
	}

	if(!(PINB & (1<<PB3)))
	{
		if(time.mins == 0)
		{
			time.mins = 59;
		}
		else
		{
		(time.mins)--;
		}

		update_display();

		while(!(PINB & (1<<PB3)))
		{show_display();}
	}

	if(!(PINB & (1<<PB6)))
	{
		if (time.secs == 59)
		{
			time.secs = 0;
		}
		else
		{
		(time.secs)++;
		}

		update_display();

		while(!(PINB & (1<<PB6)))
		{show_display();}
	}

	if(!(PINB & (1<<PB5)))
	{
		if (time.secs == 0)
		{
			time.secs = 59;
		}
		else
		{
		(time.secs)--;
		}

		update_display();

		while(!(PINB & (1<<PB5)))
		{show_display();}
	}
}

void toggle_mode()
{
	if (!(PINB & (1<<7)))
	{
		countdown_mode^=1;

		PORTD ^= (1<<4); // toggle red led
		PORTD ^= (1<<5); // toggle yellow led

		while(!(PINB & (1<<7))) {show_display();}
	}
}
int main()
{
    /* initialize pins */
    DDRA |= 0x3F;   // o/p pins for displays
    DDRB &= ~(0x7B);   // i/p pins
    DDRC |= 0x0F;   // o/p pins for decoder
    DDRD |= (1<<0) | (1<<4) | (1<<5); // o/p pins for leds

    PORTC &= 0xF0;  // initialize BCD to zero
    PORTB |= 0xFB; // internal pull ups fpr inc & dec, and toggle mode

    PORTD |= (1<<4); // red led initially on
    PORTD &= ~(1<<5); // yellow led initially off

    TIMER1_COMP_Init();
    INT0_Init();
    INT1_Init();
    INT2_Init();

    sei(); // global interrupts

    for(;;)
    {
    	show_display();

    	toggle_mode();

    	while(pause_flag==1)
    	{
    	show_display();
    	buttons_inc_dec();
    	toggle_mode();
    	}
    }
}

