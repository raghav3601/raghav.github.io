volatile uint8_t phase = 0; // motor firing phase
volatile long position = 0; // current position
volatile bool sample_and_hold_pin = false; // flag to tell loop to sample current position
volatile byte position_buffer[] = {0, 0, 0, 0}; // buffer for transfering single bytes of position over spi
volatile int  addr=0;


//ATTEMPT1
//HALL a:yellow b:blue c:green phase a:yellow b:green c:blue dir 0:anti clock(smoother running) dir 1:clock(phase misfire and abrupt)
//phase c out short with vm
//ATTEMPT2 (USE THIS ONE! WORKS GREAT!)
//HALL a:gr;b:bl;c:yellow | Phase: A:Gr B:Bl C:yellow CCW from the back @ DIR 0
//ATTEMPT3 (USE THIIS ONE ON THE GOLDEN COLORED MOTOR, KINDA WORKS)
//HALL a:bl|or;b:gr|br:c:yl|yl || Phase: A:Gr;B:Yl;C:Bl


//Mod_START

#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int pot_Val = 0; //Variable for storing the raw pot voltage input.
int PWM_Val = 0; //PWM Variable for determining the max output divisor wrt the input voltage
bool flag = 0;   //Flag to determine the control structure.
//Mod_END

void setup() {


  //Mod_START
#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2) ;
  cbi(ADCSRA, ADPS1) ;
  cbi(ADCSRA, ADPS0) ;
#endif

  setPwmFrequency(9, 8);//Was 8 on 20.10.17| setPwmFrequency(PIN,DIV) Divisor is 1: 31KHz; 8: 4KHz @ 16MHz CLKOUT OR HALF WIDTH @ 8MHz IRCOSC

  //Mod_END

  // configure motor control outputs
  DDRC |= 0x3F;  // A0-A5 as outputs
  // configure uC control pins
  DDRD &= !0xEF; // 0-3, 5-7 as inputs (will break serial communication and require ISP programmer to undo)
  //DDRD |= 0x10;  // 4 as output, used for software pin change interrupt
  pinMode(0, INPUT);  // sample and hold
  pinMode(1, INPUT);  // motor direction
  pinMode(2, INPUT);  // encoder channel B
  pinMode(3, INPUT);  // encoder channel A
  pinMode(4, OUTPUT); // used for software initiated pin change interrupt
  //pinMode(5,INPUT); //digitalWrite(5, HIGH);
  pinMode(5, INPUT_PULLUP);  // hall sensor 3
  pinMode(6, INPUT_PULLUP);  // hall sensor 2
  pinMode(7, INPUT_PULLUP);  // hall sensor 1
  pinMode(9, OUTPUT); //PWM OUT PIN;
  digitalWrite(9, LOW); //To avoid stray PWM into the NAND chopping stage.
  //pinMode(8, OUTPUT); // status LED
  pinMode(10, INPUT);  // pwm pin, master controls this
  pinMode(MISO, OUTPUT); //To enable the ISP Programming.
  //pinMode(SS, INPUT); //SS pin
  pinMode(A7, INPUT);
  /*


    SPCR |= 0b00;       // SPI speed f_osc/4 (4MHz with 16MHz chip)
    SPCR &= ~_BV(DORD); // MSB first
    SPCR &= ~_BV(MSTR); // slave mode
    SPCR &= ~_BV(CPOL); // clock low when idle
    SPCR &= ~_BV(CPHA); // sample on clock rising edge
    SPCR |=  _BV(SPE);  // enable SPI
    SPCR |=  _BV(SPIE); // enable SPI interrupts

  */

  // initialize interrupts
  initInterrupt();

  // setup phase to correct motor position by triggering pin change interrupt a few times
  // fills in hall variable in pin change ISR
  delay(1);
  digitalWrite(4, LOW);
  delay(1);
  digitalWrite(4, HIGH);
  delay(1);
  digitalWrite(4, LOW);

  // flash LED thrice on reset
  /*
     for (int i = 0; i < 6; i++) {
    PINB |= _BV(0);
    delay(80);
    }
  */
}



void loop() {
  //Mod_START
  pot_Val = analogRead(A7); //Read the pot value on ADC7
  if(pot_Val<250)pot_Val=250; else if(pot_Val>1000)pot_Val=1000; //Pot/Accelarator error correction.
  PWM_Val = map(pot_Val, 250, 1000, 0, 255);
  //PWM_Val = pot_Val / 8; //24V: 8|48V:4 - Divisor Values for PWM max width determination. Caution: VLFDCPWM can result in excessive switching heat.
  analogWrite(9, PWM_Val); //Write the PWM on Timer0
  //if(digitalRead(10)==HIGH)//13-14 |10 IS SS - can use! Mode C Selection.
  //bdc(); //Calls the Brushed DC Motor Control Solution.
  //else if(digitalRead(10)==LOW) //To ensure no false feedback.
  bldc(); // Calls the Brushless DC Motor Control Solution.
}


void initInterrupt() {

  cli();                //disable interrupts while changing settings
  PCICR |= 1 << PCIE2;  // set bit 2 in PCICR for PCINT23:16 (port D)
  PCMSK2 |= 0xFF;       // enable pin change interrupts on port D (all pins)
  sei();

}



ISR(PCINT2_vect) { // run every time there is a pin change on port D pin change

  // lookup table for position incrementing (3-phase version of quadrature)
  static int8_t hall_inc[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1,
  0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // lookup table for setting next motor phase, extended to allow reverse direction
  static uint8_t phase_lookup[] = {0, 2, 4, 3, 6, 1, 5, 2, 4, 3, 7};
  //static uint8_t phase_lookup[] = {0, 2, 4, 3, 6, 1, 5, 2, 4, 3};
  
  static int hall = 0;
  static uint8_t dir = 0;
  static bool sample_and_hold_set = false; //S&H failed and discontinued on Prototype #11.

  int port = PIND; // read the port

  sample_and_hold_pin = port & 1; // flag to tell loop to store current position in buffer

  dir = (port >> 1) & 1; // direction value, 1 or 0
  hall = hall << 3;      // preserve last read for position in/decrement
  hall |= port >> 5;     // shift to read only hall sensors - shailuch11@gmail.com

  if (dir == 0) {
    phase = phase_lookup[(hall & 0x07)];     // determine next phase to fire on the motor
  } else {
    phase = phase_lookup[(hall & 0x07) + 3]; // adding 3 to lookup index has the effect of reversing the motor (MAGIC!)
  }

 volatile long pre = position;
  
  position += hall_inc[hall & 0x3F]; // use <hall_prev><hall_current> as lookup index to which will increment, decrement or do nothing to position value

 
  
  PINB |= _BV(0);                             // toggle LED
  
  /*if(pre!=position) EEPROM.write(addr, position);
  addr+=sizeof(position);
  if(addr==EEPROM.length())addr=0;
*/
}



/*
ISR(SPI_STC_vect) { // spi handler

  byte position_byte = SPDR; // read in command from master (which byte of position to send next)
  SPDR = position_buffer[position_byte]; // send a single byte from position_buffer

}
*/


void commutate(uint8_t _phase) {

  //                                  {lowers on, Bh:Al,    Ch:Al,    Ch:Bl,    Ah:Bl,    Ah:Cl,    Bh:Cl}
  static uint8_t phase_to_port_ABC[] = {0b000000, 0b010001, 0b100001, 0b100010, 0b001010, 0b001100, 0b010100};//CH;BH;AH;CL;BL;AL -Bit Sequence
  static uint8_t phase_to_port_ACB[] = {0b000111, 0b100001, 0b010001, 0b010100, 0b001100, 0b001010, 0b100010};
  static uint8_t phase_to_port_BAC[] = {0b000111, 0b001010, 0b100010, 0b100001, 0b010001, 0b010100, 0b001100};
  static uint8_t phase_to_port_BCA[] = {0b000111, 0b001100, 0b010100, 0b010001, 0b100001, 0b100010, 0b001010};
  static uint8_t phase_to_port_CAB[] = {0b000111, 0b100010, 0b001010, 0b001100, 0b010100, 0b010001, 0b100001};
  static uint8_t phase_to_port_CBA[] = {0b000111, 0b010100, 0b001100, 0b001010, 0b100010, 0b100001, 0b010001};

  // the order of the 3 motor wires will dictate which of the above lookup tables to use.
  // TODO: be able to select one of above lookup tables from master
  PORTC = phase_to_port_ABC[_phase];

}
/*
 * CL BL AL CH BH AH SELST
 * 1  0  1  0  1  1   0
 * 1  0  1  1  1  0   0 
 * 0  1  1  1  1  0   0
 * 1  1  0  1  0  1   1
 * 0  1  1  1  0  1   1
 * 1  1  0  0  1  1   0
 */
//Mod_Func_START



void setPwmFrequency(int pin, int divisor) {
  byte mode; //Chip compatible variable for feeding timer data.
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) { //OC Pins
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) { //Watchdog Timer Pins
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void bdc()
{
  //if (digitalRead(1) == HIGH)
  PORTC = 0b010001;// else PORTC = 0b001100; //CH;BH;AH;CL;BL;AL -Bit Sequence
}


void bldc()
{
  static long sample_position = 0;
  static bool sample_and_hold_set = false;

  // sample and hold event
  if (sample_and_hold_pin == true && sample_and_hold_set == false) {
    sample_position = position;                 // sample position
    position_buffer[0] = sample_position >> 24; // divide sampled position
    position_buffer[1] = sample_position >> 16; // into byte sized chuncks
    position_buffer[2] = sample_position >> 8;  // for SPI transfer
    position_buffer[3] = sample_position;       //
    sample_and_hold_set = true;                 // set flag that sample has been taken
    PINB |= _BV(0);                             // toggle LED
  } else if (sample_and_hold_pin == false && sample_and_hold_set == true) {
    sample_and_hold_set = false;                // remove sample flag
  }

  // commutate motor
  commutate(phase);

}


//Mod_Func_END

