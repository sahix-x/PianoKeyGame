#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "serialATmega.h"
#include "spiAVR.h"
#include "LCD.h"
#include "avr/eeprom.h"
#include <stdlib.h>

#define NUM_TASKS 4 // TODO: Change to the number of tasks being used
#define HIGH_SCORE_ADDRESS 0

// Task struct for concurrent synchSMs implmentations
typedef struct _task
{
  signed char state;         // Task's current state
  unsigned long period;      // Task period
  unsigned long elapsedTime; // Time elapsed since last task tick
  int (*TickFct)(int);       // Task tick function
} task;

//  e.g. const unsined long TASK1_PERIOD = <PERIOD>
const unsigned long GCD_PERIOD = 25; // TODO:Set the GCD Period
const unsigned long BUTTON_PERIOD = 50;
const unsigned long SONG_PERIOD = 50;
const unsigned long MATRIX_PERIOD = 25;
const unsigned long LCD_PERIOD = 50;
// Global Variables
bool sysOn = 0;
int noteIndex = 0;
int duration = 0;
int durationMod = 0;
bool gameOver = 0;
bool PlayerReady = 0;
uint8_t score = 0;
uint8_t highScore;
char strBuffer[2];

task tasks[NUM_TASKS]; // declared task array with 5 tasks

// TODO: Declare your tasks' function and their states here
void TimerISR()
{
  for (unsigned int i = 0; i < NUM_TASKS; i++)
  { // Iterate through each task in the task array
    if (tasks[i].elapsedTime == tasks[i].period)
    {                                                    // Check if the task is ready to tick
      tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
      tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
    }
    tasks[i].elapsedTime += GCD_PERIOD; // Increment the elapsed time by GCD_PERIOD
  }
}

// SPI send only sends 8 bits, the matrix needs 16 bits
void maxSend(unsigned char hexAddy, unsigned char hexData)
{
  SPI_SEND(hexAddy);
  SPI_SEND(hexData);
}

void Matrix_Init();
void clearMatrix()
{
  for (int i = 0; i < 8; i++)
  {
    PORTB = SetBit(PORTB, 2, 0); // set CS low
    SPI_SEND(i + 1);
    SPI_SEND(0x00);
    PORTB = SetBit(PORTB, 2, 1);
  }
}

int Eb = 3213;
int D = 3404;
int C = 3821;
int Glow = 5101;
int Ab = 4814;
int Ghi = 2550;
int Fhi = 2862;
int Flow = 5726;
int FSharp = 5405;
int Bb = 4289;
int A = 4545;
int Rest = 0;

int NotesArray[52][2] = {
    // {Rest, 7},   // 7 * 50ms = 350ms
    {Eb, 5},     // 5 * 50ms = 250ms is an 16th note
    {D, 5},      // 5 * 50ms = 250ms
    {C, 5},      // 5 * 50ms = 250ms
    {D, 5},      // 5 * 50ms = 250ms is a sixteenth note
    {Eb, 10},    // 10 * 50ms = 500ms is a eighth note
    {Glow, 10},  // 10 * 50ms = 500ms is a eighth note
    {Ab, 5},     // 5 * 50ms = 250ms is a sixteenth note
    {Ghi, 5},    // 5 * 50ms = 250ms
    {Fhi, 5},    // 5 * 50ms = 250ms
    {Eb, 5},     // 5 * 50ms = 250ms
    {D, 5},      // 5 * 50ms = 250ms
    {C, 5},      // 5 * 50ms = 250ms
    {D, 5},      // 5 * 50ms = 250ms
    {Eb, 5},     // 5 * 50ms = 250ms
    {Fhi, 10},   // 10 * 50ms = 500ms
    {Flow, 10},  // 10 * 50ms = 500ms
    {Glow, 10},  // 10 * 50ms = 500ms
    {Fhi, 10},   // 10 * 50ms = 500ms
    {Eb, 10},    // 10 * 50ms = 500ms
    {D, 10},     // 10 * 50ms = 500ms
    {C, 5},      // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {Ab, 5},     // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {C, 20},     // 20 * 50ms = 1000ms is a quarter note
    {C, 5},      // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {A, 5},      // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {C, 20},     // 20 * 50ms = 1000ms
    {C, 5},      // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {Ab, 5},     // 5 * 50ms = 250ms
    {Bb, 5},     // 5 * 50ms = 250ms
    {C, 5},      // 5 * 50ms = 250ms
    {Flow, 5},   // 5 * 50ms = 250ms
    {FSharp, 5}, // 5 * 50ms = 250ms
    {Eb, 5},     // 5 * 50ms = 250ms
    {D, 20},     // 20 * 50ms = 1000ms
    {Rest, 10},  // 10 * 50ms = 500ms
    {D, 10},     // 10 * 50ms = 500ms
    {Eb, 5},     // 5 * 50ms = 250ms
    {D, 5},      // 5 * 50ms = 250ms
    {C, 5},      // 5 * 50ms = 250ms
    {D, 5},      // 5 * 50ms = 250ms
    {Eb, 5},     // 5 * 50ms = 250ms
    {Fhi, 5},    // 5 * 50ms = 250ms
    {Ghi, 5},    // 5 * 50ms = 250ms
    {Glow, 5},   // 5 * 50ms = 250ms
    {Ab, 10},    // 5 * 50ms = 250ms
    {Ghi, 10},   // 10 * 50ms = 500ms
    {Fhi, 20},   // 20 * 50ms = 1000ms
};
char matrixRow[8]; // handles the rows
enum System_states
{
  SystemStart,
  ButtonWait,
  onPress,
  onRelease,
  offPress,
};
int System_Button(int SysButtState);

enum Song_states
{
  SongStart,
  SongWait,
  SongPlay,
};
int Song_Tick(int SongState);

enum Matrix_states
{
  Matrix_start,
  Matrix_wait,
  Matrix_display,
};
int Matrix_Tick(int MatrixState);

enum LCD_states
{
  LCD_INIT,
  LCD_Start,
  LCD_Countdown,
  LCD_Game,
  LCD_GameOver,
};

int LCD_Tick(int LCDState);

int main(void)
{
  // TODO: initialize all your inputs and ouputs
  DDRC = 0x00;  // Set all pins on PORTC as inputs
  PORTC = 0xFF; // Enable pull-ups on PORTC

  DDRB = 0xFF;  // Set all pins on PORTB as outputs
  PORTB = 0x00; // Initialize PORTB

  DDRD = 0xFF;  // Set all pins on PORTD as outputs
  PORTD = 0x00; // Initialize PORTD

  // Initialize Timer1 for PWM for buzzer
  //  configures the timer to use Channel A (pin 9) for PWM.
  TCCR1A |= (1 << WGM11) | (1 << COM1A1); // COM1A1 sets it to channel A

  // enables "fast PWM mode" with a prescaler of 8.
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // CS11 sets the prescaler to be 8
  // WGM11, WGM12, WGM13 set timer to fast pwm mode

  ADC_init(); // initializes ADC

  TimerSet(GCD_PERIOD);
  TimerOn();
  SPI_INIT(); // initialize SPI
  clearMatrix();
  Matrix_Init();
  // lcd_init(); // initialize LCD screen

  highScore = eeprom_read_byte(HIGH_SCORE_ADDRESS);
  if (highScore == 0xFFFF)
  {
    eeprom_write_byte(HIGH_SCORE_ADDRESS, 0);
  }

  serial_init(9600);

  unsigned char i = 0;
  tasks[i].state = SystemStart;
  tasks[i].period = BUTTON_PERIOD;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &System_Button;
  ++i;

  tasks[i].state = SongStart;
  tasks[i].period = SONG_PERIOD;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &Song_Tick;
  ++i;

  tasks[i].state = Matrix_start;
  tasks[i].period = MATRIX_PERIOD;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &Matrix_Tick;

  ++i;
  tasks[i].state = LCD_INIT;
  tasks[i].period = LCD_PERIOD;
  tasks[i].elapsedTime = tasks[i].period;
  tasks[i].TickFct = &LCD_Tick;
  while (1)
  {
    // serial_println(sysOn, 10);
  }

  return 0;
}

void Matrix_Init()
{
  // Decode Mode Off
  PORTB = SetBit(PORTB, 2, 0); // set CS low
  SPI_SEND(0x09);
  SPI_SEND(0x00);
  PORTB = SetBit(PORTB, 2, 1); // set CS high

  // Setting the intensity low temporarily
  PORTB = SetBit(PORTB, 2, 0); // set CS low
  SPI_SEND(0x0A);
  SPI_SEND(0x0F);
  PORTB = SetBit(PORTB, 2, 1); // set CS high

  // Scan Limit to all
  PORTB = SetBit(PORTB, 2, 0); // set CS low
  SPI_SEND(0x0B);
  SPI_SEND(0x07);
  PORTB = SetBit(PORTB, 2, 1); // set CS high

  // Shutdown to on
  PORTB = SetBit(PORTB, 2, 0); // set CS low
  SPI_SEND(0x0C);
  SPI_SEND(0x01);
  PORTB = SetBit(PORTB, 2, 1); // set CS high

  // Display test to off temporarily
  PORTB = SetBit(PORTB, 2, 0); // set CS low
  SPI_SEND(0x0F);
  SPI_SEND(0x00);
  PORTB = SetBit(PORTB, 2, 1); // set CS high
}

int System_Button(int SysButtState)
{
  // State Transitions
  switch (SysButtState)
  {
  case SystemStart:
    sysOn = 0;
    SysButtState = ButtonWait; // Return the new state instead of assigning it

  case ButtonWait:
    if (GetBit(PINC, 5)) // Button is pressed move state
    {
      SysButtState = onPress;
    }
    else // Button is not pressed -> stay in state
    {
      SysButtState = ButtonWait;
    }
    break;

  case onPress:
    if (!GetBit(PINC, 5)) // If button is released
    {
      sysOn = 1;
      SysButtState = onRelease;
    }
    else if (GetBit(PINC, 5)) // If button stays pressed
    {
      SysButtState = onPress;
    }
    break;

  case onRelease:
    if (GetBit(PINC, 5)) // If the button is pressed, change states AKA start turn off process
    {
      SysButtState = offPress;
    }
    else // If button stays released stay in current State
    {
      SysButtState = onRelease;
    }
    break;

  case offPress:
    if (!GetBit(PINC, 5)) // If the button is released turn off the system
    {
      sysOn = 0;
      SysButtState = ButtonWait;
    }
    else // Otherwise button is still being Pressed
    {
      SysButtState = offPress;
    }
    break;

  default:
    SysButtState = SystemStart;
    break;
  }

  // State Actions
  switch (SysButtState)
  {
  case SystemStart:
    break;

  case ButtonWait:
    break;

  case onPress:
    break;

  case onRelease:
    break;

  case offPress:
    break;
  }

  return SysButtState;
}
int calcButton()
{
  return (noteIndex % 4);
}
int Song_Tick(int SongState)
{
  // State Transitions
  switch (SongState)
  {
  case SongStart:
    highScore = eeprom_read_byte(HIGH_SCORE_ADDRESS);
    if (score > highScore)
    {
      highScore = score;
      eeprom_write_byte(HIGH_SCORE_ADDRESS, highScore);
    }

    serial_println(eeprom_read_byte(HIGH_SCORE_ADDRESS), 10);

    ICR1 = 0; // Turn off sound after playing
    OCR1A = 0;
    noteIndex = 0;
    score = 0;
    // gameOver = 0;
    SongState = SongWait;
    break;

  case SongWait:
    if (sysOn && PlayerReady) // add a variable to check if the countdown is over (nested if statement)
    {
      SongState = SongPlay;
    }
    else if (!sysOn && !PlayerReady)
    {
      SongState = SongWait;
    }
    break;

  case SongPlay:
    if (sysOn && (noteIndex < 52) && (!gameOver))
    {
      SongState = SongPlay;
    }
    else if (!sysOn || (noteIndex >= 52) || gameOver)
    {
      sysOn = 0;
      noteIndex = 0;
      gameOver = 0;
      PlayerReady = 0;
      SongState = SongStart;
    }
    break;

  default:
    SongState = SongStart;
    break;
  }
  // State Actions
  switch (SongState)
  {
  case SongStart:
    break;

  case SongWait:
    ICR1 = 0; // Turn off sound after playing
    OCR1A = 0;
    break;

  case SongPlay:
    duration++;
    // serial_println(score, 10);
    // For testing purposes
    if (GetBit(PINC, calcButton()) && duration < NotesArray[noteIndex][1] && NotesArray[noteIndex][0] != Rest)
    {
      gameOver = 1;
    }

    // uint8_t wrongButtonPressed = 0;

    // // Check all 4 buttons (pins 0-3)
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //   if (i != calcButton() && GetBit(PINC, i))
    //   {
    //     wrongButtonPressed = 1;
    //     break;
    //   }
    // }

    // // Game over if wrong button is pressed or if correct button is pressed at wrong time
    // if ((wrongButtonPressed || GetBit(PINC, calcButton())) &&
    //     duration < NotesArray[noteIndex][1] &&
    //     NotesArray[noteIndex][0] != Rest)
    // {
    //   gameOver = 1;
    // }

    if (duration >= NotesArray[noteIndex][1])
    {
      duration = 0;
      noteIndex++;
      score++; // check if the correct score
    }
    ICR1 = NotesArray[noteIndex][0]; // pwm period depending on the note frequency
    OCR1A = NotesArray[noteIndex][0] / 2;
    break;
  }
  return SongState;
}

int Matrix_Tick(int MatrixState)
{
  static int currentNote = -1;    // Tracks the current note being displayed
  static int currentDuration = 0; // Tracks how many ticks the current note has been displayed
  static int noteColumn = 0;      // The column for the current note
  static int ledMatrix[8];        // Represents the LED matrix state

  // State Transitions
  switch (MatrixState)
  {
  case Matrix_start:
    currentNote = -1;
    currentDuration = 0;
    noteColumn = 0;
    for (int i = 0; i < 8; i++)
    {
      ledMatrix[i] = 0;
    }
    MatrixState = Matrix_wait;
    break;

  case Matrix_wait:
    if (sysOn)
    {
      MatrixState = Matrix_display;
    }
    break;

  case Matrix_display:
    if (!sysOn)
    {
      MatrixState = Matrix_wait;
    }
    break;

  default:
    MatrixState = Matrix_start;
    break;
  }

  // State Actions
  switch (MatrixState)
  {
  case Matrix_start:
    // Initialize matrix state
    for (int i = 0; i < 8; i++)
    {
      ledMatrix[i] = 0;
    }
    currentNote = -1;
    currentDuration = 0;
    break;

  case Matrix_wait:
    // Clear the matrix when waiting
    clearMatrix();
    break;

  case Matrix_display:

    // If no current note is active, load the next one
    if (currentNote == -1 && noteIndex < 52)
    {
      // durationMod = 0;
      currentNote = noteIndex;
      currentDuration = 0;
      noteColumn = calcButton() * 2 + 1; // Map to columns 1, 3, 5, or 7
      // We multiply by 2 because after %, we get 0, 1, 2, 3, multiply by 2 to get 0, 2, 4, 6
      // Then add 1 to get 1, 3, 5, 7 for the desired column
    }

    // Clear the matrix for the new frame
    for (int i = 0; i < 8; i++)
    {
      ledMatrix[i] = 0;
    }
    // If a note is active, display it
    if (currentNote != -1)
    {
      durationMod = (((NotesArray[currentNote][1]) / 5) * 2);

      for (int row = 0; row < 8; row++)
      {
        if ((row < durationMod / 2) && (NotesArray[currentNote][0] != Rest))
        {
          ledMatrix[row] |= (1 << (8 - noteColumn)); // Light up the specific column
        }
      }
      // Increment the duration counter
      currentDuration++;

      // If the note duration is complete, move to the next note
      // Or if the current duration is greater than the number of rows, reset the note (not needed?)
      if (currentDuration > durationMod || currentDuration >= 8)
      {
        currentNote = -1; // Reset the current note
        currentDuration = 0;
      }
    }
    // Send updated matrix to the LED hardware
    for (int row = 0; row < 8; row++)
    {
      PORTB = SetBit(PORTB, 2, 0); // Set CS low
      SPI_SEND(row + 1);           // Send row address (1-based)
      SPI_SEND(ledMatrix[row]);    // Send column data for the row
      PORTB = SetBit(PORTB, 2, 1); // Set CS high
    }
    break;
  }

  return MatrixState;
}

int LCD_Tick(int LCDState)
{
  static int countdown = 60;
  // State Transitions
  switch (LCDState)
  {
  case LCD_INIT:
    ICR1 = 0;
    OCR1A = 0;
    PlayerReady = 0;
    LCDState = LCD_Start;
    break;

  case LCD_Start:
    if (sysOn) // Transition to countdown only if sysOn is explicitly triggered
    {
      countdown = 60; // Reset countdown
      LCDState = LCD_Countdown;
    }
    else
    {
      LCDState = LCD_Start;
    }
    break;

  case LCD_Countdown:
    if (countdown <= 0)
    {
      countdown = 1;   // Prevent underflow
      PlayerReady = 1; // Set flag indicating readiness
      LCDState = LCD_Game;
    }
    else
    {
      LCDState = LCD_Countdown;
    }
    break;

  case LCD_Game:
    if (gameOver)
    {
      LCDState = LCD_GameOver;
    }
    else
    {
      LCDState = LCD_Game;
    }
    break;

  case LCD_GameOver:
    if (countdown <= 0)
    {
      ICR1 = 0;
      OCR1A = 0;
      LCDState = LCD_INIT;
    }
    else
    {
      LCDState = LCD_GameOver;
    }
    break;

  default:
    LCDState = LCD_INIT;
    break;
  }

  // State Actions
  switch (LCDState)
  {
  case LCD_INIT:
    break;

  case LCD_Start:
    // lcd_clear();

    // lcd_create_custom_char(0, sixteenth_note);
    // lcd_goto_xy(0, 0);
    // lcd_write_character(0);

    // lcd_create_custom_char(1, eighth_note2);
    // lcd_goto_xy(0, 1);
    // lcd_write_character(1);

    // lcd_goto_xy(0, 3);
    // lcd_write_str("A5 to Play");

    // lcd_create_custom_char(3, sixteenth_note2);
    // lcd_goto_xy(0, 14);
    // lcd_write_character(3);

    // lcd_create_custom_char(4, sixteenth_note);
    // lcd_goto_xy(0, 15);
    // lcd_write_character(4);
    // break;
    break;

  case LCD_Countdown:
    // lcd_clear();

    // lcd_create_custom_char(0, sixteenth_note);
    // lcd_goto_xy(0, 0);
    // lcd_write_character(0);

    // lcd_create_custom_char(1, eighth_note2);
    // lcd_goto_xy(0, 1);
    // lcd_write_character(1);

    // lcd_goto_xy(0, 3);
    // lcd_write_str("Score:");
    // lcd_goto_xy(0, 10);

    // lcd_write_str(itoa(score, strBuffer, 10));

    // lcd_create_custom_char(3, sixteenth_note2);
    // lcd_goto_xy(0, 14);
    // lcd_write_character(3);

    // lcd_create_custom_char(4, sixteenth_note);
    // lcd_goto_xy(0, 15);
    // lcd_write_character(4);
    if (countdown > 40)
    {
      ICR1 = 2272; // Frequency for "3"
      OCR1A = 2272 / 2;
    }
    else if (countdown > 20)
    {
      ICR1 = 3029; // Frequency for "2"
      OCR1A = 3029 / 2;
    }
    else if (countdown > 0)
    {
      ICR1 = 4544; // Frequency for "1"
      OCR1A = 4544 / 2;
    }
    else if (countdown == 0)
    {
      ICR1 = 0; // Stop sound
      OCR1A = 0;
      PlayerReady = 1; // Set PlayerReady flag
    }

    countdown--; // Decrement countdown in Countdown state
    break;

  case LCD_Game:
    // Game LCD display updates
    break;

  case LCD_GameOver:
    ICR1 = 2272;
    OCR1A = 2272 / 2;
    countdown--; // Decrement countdown in GameOver state
    // lcd_clear();

    // lcd_create_custom_char(0, sixteenth_note);
    // lcd_goto_xy(0, 0);
    // lcd_write_character(0);

    // lcd_create_custom_char(1, eighth_note2);
    // lcd_goto_xy(0, 1);
    // lcd_write_character(1);

    // lcd_goto_xy(0, 3);
    // lcd_write_str("HiScore:");
    // lcd_goto_xy(0, 11);

    // lcd_write_str(itoa(highScore, strBuffer, 10));

    // lcd_create_custom_char(3, sixteenth_note2);
    // lcd_goto_xy(0, 14);
    // lcd_write_character(3);

    // lcd_create_custom_char(4, sixteenth_note);
    // lcd_goto_xy(0, 15);
    // lcd_write_character(4);

    break;
  }

  return LCDState;
}
