// TrafficLight.c
// Runs TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Your Name:
// created: 
// last modified: 
// 

// include headers 

#include "tm4c123gh6pm.h"
#include "SysTick.c"
#include "PLL.h"

//#include <stdint.h>

#define SYSCTL_RCGC2_R					(*((volatile unsigned long *)0x400FE108))
#define SENSOR                  (*((volatile unsigned long *)0x400243FC))//SENSOR AS THE PORTE DATA REGISTER FOR INPUTS
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

#define SYSCTL_RCGC2_R					(*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC)) //DATAR FOR PORT B WILL BE THE OUTPUT LIGBTS
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))

struct state{
	unsigned long OUT;
  unsigned long TIME;
	struct state *Next[16]; // THIS INDICATES THAT THERE WILL BE 16 POINTERS OUT OF THIS STRUCTURE 
};
typedef struct state stype;

	#define DRIVEIVY &FSM[0] 
	#define WAITW1ST &FSM[1] 
	#define WAITIVY &FSM[2] 
	#define DRIVEW1ST &FSM[3] 
	#define WALKIVY &FSM[4] 
	#define WALKW1STandDRIVEIVY &FSM[5] 
	#define WALKBOTH &FSM[6] 
	#define WALKW1ST &FSM[7] 

stype FSM[8]={ //state type with 8 states, each state has the nxt state to go to based on a certain input read by the sensor 
{0x21,350000,{DRIVEIVY,DRIVEIVY,WAITIVY,DRIVEIVY,WAITIVY,DRIVEIVY,WAITIVY,DRIVEIVY,WALKW1STandDRIVEIVY,WALKW1STandDRIVEIVY,WAITIVY,DRIVEIVY,WAITIVY,DRIVEIVY,WAITIVY,DRIVEIVY}},
{0x14,100000,{DRIVEIVY,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKIVY,WALKIVY,WALKIVY,WALKIVY,WALKW1ST,WALKW1STandDRIVEIVY,WALKW1ST,WALKW1ST,WAITW1ST,WALKBOTH,WALKBOTH,WALKBOTH}},
{0x22,100000,{DRIVEW1ST,DRIVEW1ST,DRIVEW1ST,DRIVEW1ST,WALKIVY,WALKIVY,WALKIVY,WALKBOTH,WALKW1ST,WALKW1ST,WALKW1ST,WALKBOTH,WALKBOTH,WALKBOTH,WALKBOTH,WALKBOTH}},
{0x0C,350000,{DRIVEW1ST,WAITW1ST,DRIVEW1ST,WAITW1ST,WAITW1ST,WAITW1ST,DRIVEW1ST,WAITW1ST,WAITW1ST,WAITW1ST,DRIVEW1ST,WAITW1ST,WAITW1ST,WAITW1ST,DRIVEW1ST,WAITW1ST}},
{0xA4,200000,{WALKIVY,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY}},
{0x61,350000,{WALKW1STandDRIVEIVY,WALKW1STandDRIVEIVY,WAITW1ST,WALKW1STandDRIVEIVY,WAITIVY,WAITIVY,WAITIVY,WAITIVY,WALKW1STandDRIVEIVY,WALKW1STandDRIVEIVY,WAITIVY,WAITIVY,WAITIVY,WAITIVY,WAITIVY,WAITIVY}},
{0xE4,200000,{WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY}},
{0x64,200000,{WALKW1ST,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,WALKW1STandDRIVEIVY,DRIVEW1ST,DRIVEIVY,WALKBOTH,DRIVEIVY,DRIVEW1ST,DRIVEIVY}}
};
//ABOVE LINES 56-63 ESSENTIALLY INDICATE THAT WHEN WE HAVE A CERTAIN INPUT...LIKE 0X21 FOR EXAMPLE
// OUTPUT STATE, TIME IN STATE, {ALL NEXT STATES IN NUMERICAL ORDER 0X00-0XFF} 
// THE NUMERICAL ORDER CAN REFERENCED IN THE FSM TABLE
//THE NUMERICAL ORDER CORRESPONDS TO EACH INPUT
// A 1-1 MAPPING OF THE STATE GRAPH, TABLE, AND WRITTEN SOFTWARE
// START AT THE TABLE, VIEW IN DIAGRAM, DEVELOP IN THE SOFTWARE
//we are covering all possible inputs on this system when designing out FSM

// Linked data structure
//define your states here e.g. #define stateName 0, etc.
//Declare your states here 
void PortE_Init(void);
void PortB_Init(void);
//unsigned long input;

int main(void){
//volatile unsigned long delay;
	
stype *Pt;
unsigned long input;	
PortE_Init();
PortB_Init();
PLL_Init();
SysTick_Init();

Pt = DRIVEIVY; // STATE WE ARE STARTING OUT IN....ARBITRARILY SET 
	 
//unsigned long input;
	  
while(1){

GPIO_PORTB_DATA_R = Pt->OUT; //read gpio portb data r as the output (LED lights) 
SysTick_Wait(Pt->TIME); //use arrows to access elements within a structure 
input = GPIO_PORTE_DATA_R; //read gpio port e data r as input (sensor)
Pt = Pt->Next[input]; // MEANING GO TO NEXT FEILD BASED ON THE NEXT INPUT 

  }
}
	
void PortE_Init(void){
	int x = 0;
	SYSCTL_RCGC2_R |= 0x10; //setting the clock for pF in position 0x20
	x++;
	GPIO_PORTE_LOCK_R = 0x4C4F434B; //is this actually needed? only thought for portF
	GPIO_PORTE_CR_R |= 0x1E;
	GPIO_PORTE_PCTL_R = 0x1E; //enabling all pins on pF for i/o control 
	GPIO_PORTE_PUR_R |= 0x1E;
	GPIO_PORTE_DEN_R = 0x1E; // enabling all pins on pF for digital interfacing 
	GPIO_PORTE_DIR_R &= ~0x1E;
	GPIO_PORTE_AFSEL_R &= ~0x1E; // disabling all alternate functions for pF
	GPIO_PORTE_AMSEL_R &= ~0x1E; 

}
void PortB_Init(void){
	int x = 0;
	SYSCTL_RCGC2_R |= 0x02; //set clock for portA timer
	x++;
	GPIO_PORTB_DEN_R |= 0xFF; //enable all pins for digital i/o
	GPIO_PORTB_DIR_R |= 0xFF; //we are using pa5 for output
	GPIO_PORTB_AFSEL_R &= ~0xFF;//disabling all alternate functions
	GPIO_PORTB_AMSEL_R &= ~0xFF; 
}
