#define PART_TM4C123GH6PM
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"


//enum for North states
typedef enum {
    NorthRed, 
    NorthYellow,
    NorthGreen
}North_States;

North_States Current_North_State = NorthRed;
static int Traffic_Time = 0x4C4B3FF;

//enum for East states
typedef enum {
    EastRed, 
    EastYellow,
    EastGreen
}East_States;

East_States Current_East_State = EastRed;

void East_Original_State(void);
void North_Original_State(void);
void Normal_Traffic_Light(void);
void North_Pedestrian_Interrupt_TimerFinish(void);
void North_Pedestrian_Interrupt(void);
void East_Pedestrian_Interrupt_TimerFinish(void);
void East_Pedestrian_Interrupt(void);
void PortCInt_Init(void);
void PortEInt_Init(void);
void Timer0A_INIT(void);
void Timer2A_INIT(void);
void Timer1A_INIT(void);
void PinsInit(void);
void Uart_Init(void);
void printString(char * string);
void printChar(char c);


void main(void)
{       PinsInit();
        Uart_Init();
        PortEInt_Init();
        PortCInt_Init();   
        Timer0A_INIT();   
        Timer1A_INIT();
        Timer2A_INIT();
        
        while(1){
       }      
   	
}

//switch case function for states
void North_Original_State(){
  switch (Current_North_State)
 {
    case NorthRed:
      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0); // North is Red
      break;

    case NorthYellow:
      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1); // North is Yellow
      break;
      
    case NorthGreen:
      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2); // North is Green
    break;

  }
}


//switch case function for states
void East_Original_State(){
   switch (Current_East_State)
 {
    case EastRed:
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // East is Red
      break;

    case EastYellow:
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // East is Yellow
      break;
      
    case EastGreen:
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // East is Green
    break;

  }
}


void Normal_Traffic_Light(void)
{
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        
        TimerDisable(TIMER0_BASE, TIMER_A);   
	static int i = 0;
	
          switch(i){
                  case 0:
			i++;
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2); // North is Green
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // East is RED
                        
                        Current_North_State = NorthGreen;
                        Current_East_State = EastRed;
                        printString("NorthGreen");
                        
                        
                        Traffic_Time = 0x4C4B3FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);
			// 5 seconds delay
			
                        break;
                case 1:
                        i++;     
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1); // North is Yellow
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // East is RED
                        
                        Current_North_State = NorthYellow;
                        Current_East_State = EastRed;
                        printString("NorthYellow");
                        
                        // 2 seconds delay
                        Traffic_Time = 0x1E847FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);			
			         
                        break;
               case 2:
                        i++;  
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0); // North is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // East is RED
                        
                        Current_North_State = NorthRed;
                        Current_East_State = EastRed;
                        printString("NorthRed");
                        
                        // 1 seconds delay
                        Traffic_Time = 0xF423FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);			
			
                        break;
              case 3:
                        i++;
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0); // North is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // East is Green
                        
                        Current_North_State = NorthRed;
                        Current_East_State = EastGreen;
                         printString("EastGreen");
                        
			// 5 seconds delay
                        Traffic_Time = 0x4C4B3FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);
                        			
                        break;
              case 4:
                        i++;
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0); // North is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // East is Yellow
                        
                        Current_North_State = NorthRed;
                        Current_East_State = EastYellow;
                        printString("EastYellow");
                        
			// 2 seconds delay
                        Traffic_Time = 0x1E847FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);                      
			
                        break;
              case 5:
                        i = 0;
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);  // North is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // East is RED
                        
                        Current_North_State = NorthRed;
                        Current_East_State = EastRed;
                        printString("EastRed");
                        
			// 1 seconds delay
                        Traffic_Time = 0xF423FF;
                        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);
                        
                        break;              
   		
}
 printString("\n\r");
 TimerEnable(TIMER0_BASE, TIMER_A);
}


void printString(char * string){
  while(*string){
    printChar(*(string++)); //increment the address where pointer is pointing at, while loop stops when it reaches character 0    
  }
}

void printChar(char c){
  UARTCharPut(UART0_BASE,c);
}

  
//when switch one is pressed call this as interrupt, set it in vector table
void North_Pedestrian_Interrupt(void)
{
	GPIO_PORTC_ICR_R |= 0x11; /* clear the interrupt flag before return */
	TimerDisable(TIMER0_BASE, TIMER_A);
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        Traffic_Time = TimerValueGet(TIMER0_BASE, TIMER_A);        //get ticks left

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0,GPIO_PIN_0); // East is RED
        
	//set pedestrian light to green for 2 seconds
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0);               // north pedestrian red off
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6,GPIO_PIN_6);     //  north pedestrian green on
        printString("North Pedestrian");
        printString("\n\r");
        // 2 seconds delay with timer_2A
        TimerLoadSet(TIMER2_BASE, TIMER_A,0x1E847FF);
       TimerIntRegister(TIMER2_BASE,TIMER_A,North_Pedestrian_Interrupt_TimerFinish);
        TimerEnable(TIMER2_BASE, TIMER_A);

}

void North_Pedestrian_Interrupt_TimerFinish(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER2_BASE, TIMER_A);
        
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,GPIO_PIN_3);  // north pedestrian red on
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6,0);           // north pedestrian green off
        
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
        East_Original_State();                                  //return to original state
        North_Original_State(); 
        
        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time); //ticks left
        TimerEnable(TIMER0_BASE, TIMER_A);
} 

//when switch two is pressed call this as interrupt, set it in vector table
void East_Pedestrian_Interrupt(void)
{       
        GPIO_PORTE_ICR_R |= 0x11; /* clear the interrupt flag before return */
	TimerDisable(TIMER0_BASE, TIMER_A);
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        Traffic_Time = TimerValueGet(TIMER0_BASE, TIMER_A);        //get ticks left
        
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0,GPIO_PIN_0); // North is RED
        printString("East Pedestrian");
         printString("\n\r");
        //set pedestrian light to green for 2 seconds
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3,0);              // East pedestrian red off
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4,GPIO_PIN_4);     // East pedestrian green on
        
        // 2 seconds delay with timer_1A
         
        TimerLoadSet(TIMER1_BASE, TIMER_A,0x1E847FF);
        TimerIntRegister(TIMER1_BASE,TIMER_A,East_Pedestrian_Interrupt_TimerFinish);
        TimerEnable(TIMER1_BASE, TIMER_A);
     
       
}

void East_Pedestrian_Interrupt_TimerFinish(void)
{
  
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER1_BASE, TIMER_A);
        
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3,GPIO_PIN_3);  // north pedestrian red on
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4,0);           // north pedestrian green off

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);     //turn all north leds off before configuring it
        North_Original_State();   
        East_Original_State() ;                                             //return to original state        
                       
        TimerLoadSet(TIMER0_BASE, TIMER_A,Traffic_Time);
        TimerEnable(TIMER0_BASE, TIMER_A);
}

// portC  switch init for north Pedestrian Crossing
void PortCInt_Init(void)
{
  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));    
    
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);  // Init PC4 as input, switch 1
    
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);      // Clear pending interrupts for PC4
    GPIOIntRegister(GPIO_PORTC_BASE,North_Pedestrian_Interrupt);     //handler function for port C
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4); 
} 


// portE switch init for east Pedestrian Crossing east

void PortEInt_Init(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));    
    
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);  
    
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);      
    GPIOIntRegister(GPIO_PORTE_BASE,East_Pedestrian_Interrupt);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0); 
} 



void Timer0A_INIT(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //connect clock to TIMER0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_A_PERIODIC ); //half-width, periodic timer 16 bit 
    TimerEnable(TIMER0_BASE, TIMER_A);
          
   TimerLoadSet(TIMER0_BASE, TIMER_A,1000); 
   TIMER0_TAPR_R = 0;     
   TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);   // TIMER0_IMR_R = 0x00000001;  arm timeout interrupt
   TimerIntRegister(TIMER0_BASE,TIMER_A,Normal_Traffic_Light);    // timer0A interrupt function
   IntPrioritySet(INT_TIMER0A, 0x00); // highest prority
  
}
// switch C
void Timer2A_INIT(){
   SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); 
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)){}
   TimerDisable(TIMER2_BASE, TIMER_A);
   TimerConfigure(TIMER2_BASE,TIMER_CFG_A_PERIODIC ); //half-width, periodic timer 16 bit   
   TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); 
   TIMER0_TAPR_R = 0;     
  
}
//switch E
void Timer1A_INIT(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)){}
    TimerDisable(TIMER1_BASE, TIMER_A);
    TimerConfigure(TIMER1_BASE,TIMER_CFG_A_PERIODIC ); 
    
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); 
    TIMER0_TAPR_R = 0;     
  
}


//All Leds init
void PinsInit(void)
{
	// Enable Peripheral Clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure GPIO Inputs
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4);

	// Configure GPIO Outputs
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);  //north red
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);  //north yellow
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);  //north green
        
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);  // north pedestrian red
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);  // north pedestrian green
        
        
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); // east pedestrian red
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); // east pedestrian green
        
        
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);  //east red
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);  //east yellow
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);  //east green
        
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,GPIO_PIN_3);      
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3,GPIO_PIN_3); 
}

void Uart_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    //Use the internal 16MHz oscillator as the UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    
    UARTDisable(UART0_BASE);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |UART_CONFIG_WLEN_8));;//SysCtlClockGet() -> 16000000 //parity none, one stop bit, 8 bits
    
}