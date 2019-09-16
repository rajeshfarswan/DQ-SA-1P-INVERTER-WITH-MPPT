//****************************************************************************************//
// STAND ALONE SINGLE PHASE INVERTER BASED ON SINE PWM AND UNBALANCE DQ CONTROL WITH MPPT //
//****************************************************************************************//

//include files
#include "main.h"    // Include file containing processor registors definitions and function definitions
#include "user.h"    // Include definitions of local variables
#include "asmMath.h" // Include definition of math functions 

int main(void) //main
{

init(); // call processor initilisation code
 
starting(); // Before-PWM initiliasation and soft-start of system

            //set initial MPPT parameters
         asm("disi #0x3FFF"); 
           {
          Vpv = asmADC(0x0007);      //PV voltage adc channel 7
          Ipv = asmADC(0x000A);      //PV current adc channel 10  
            } 
	     Pold = asmINT_MPQ(Vpv,Ipv); //PV power
         Vpv_old = Vpv;
           //

  while(1) //main loop 
	{

//current control//
    if(current_Flag) //20Khz
       {  
          //generate current references for DQ frame //inverter voltage d-q PI output is current ref   
          Dvalue = Vd_FOFout; //set inverter current ref d
          Qvalue = Vq_FOFout; //set inverter current ref q
          asmDQtoABC();       //inverter current refrences to dq to abc 

           //read inverter and battery currents
          asm("disi #0x3FFF");
           {
            Ivalue = asmADC(0x0b0b) - offset; // adc channel 11// Inverter current
           } 
          asm("disi #0x0000");
           //
           //detect inverter peak current
          if(Ivalue > current_max) fault_Flag = 0; //set if fault
          if(Ivalue < current_min) fault_Flag = 0;  
         
            //Inverter current PI
            IPreError = currentP_Dout;
            currentP_Dout = asmPIcontroller(Avalue,Ivalue,current_Pgain,current_Igain); //set inverter current to ref

            asmPWM(); //generate Full-bridge inverter and MPPT Converter duty cycle
                      // dyty 1 and 2 for inverter, duty 3 for PV converter
            
                               current_Flag = 0;   
                                  } 
//current control//

//Inverter voltage control start//       
	if(pll_Flag) //12Khz _PLL_count
		{
            asmDClink(); //monitor dc link

            IPreError = MPPT_I_PreOut;   // PI control for PV voltage MPPT // ref from PO MPPT routine
            MPPT_PIout = asmPIcontroller(PVref,Vpv,MPPT_Kp,MPPT_Ki); 

            IPreError = noMPPT_I_PreOut; // PI control for no-MPPT in case of excess power from PV
            noMPPT_PIout = asmPIcontroller(VDC_ref,VDC,noMPPT_Kp,noMPPT_Ki); 

            //set PV converter duty cycle
            if(noMPPT_PIout < MPPT_PIout) final_ref = noMPPT_PIout; //if dc-link overvoltage in no-MPPT mode
                  else final_ref = MPPT_PIout;                      //if normal operation in MPPT mode

           //generate dq references for inverter output voltage control
           asm("disi #0x3FFF");
           Avalue = asmINT_MPQ(qSin,V_ref); // generate inverter voltage ref from sine ref
           asm("disi #0x0000");
           asmABCtoDQ();    //references to d-q frame
           VD_ref = Dvalue; //inverter voltage d-ref
           VQ_ref = Qvalue; //inverter voltage q-ref
           //

          asm("disi #0x3FFF");              //read inverter output voltage  
           {
          Avalue = asmADC(0x0a0a) - offset; //adc channel 5 
            } 
          asm("disi #0x0000");
          //

          asmABCtoDQ(); // Inverter output voltage feedback to d-q frames

          /*Inverter voltage D PI******************************************************/
          IPreError = Vd_PI_out;
          Vd_PI_out = asmPIcontroller(VD_ref,Dvalue,vPI_Pgain,vPI_Igain);  //set inverter output voltage to d ref
           
         /*voltage Q PI******************************************************/
          IPreError = Vq_PI_out;
          Vq_PI_out = asmPIcontroller(VQ_ref,Qvalue,vPI_Pgain,vPI_Igain); //set inverter output voltage to q ref
             
          //inverter voltage D PI filter
          FOF_PreOut = Vd_FOFout;
          Vd_FOFout = asmFO_Filter(Vd_PI_out,Filter_const_V);

          //inverter voltage Q PI filter
          FOF_PreOut = Vq_FOFout;
          Vq_FOFout = asmFO_Filter(Vq_PI_out,Filter_const_V);
         
				         pll_Flag = 0;
						  }
//Voltage control//	


//inverter output voltage soft start//
		if(ffd_Flag) //0.5Khz   
      		{  
              SET = 0x0077;                   //all three switces are enabled

              V_ref++;                        //initiate soft start //update ref value

                   if(V_ref >= V_Dsetpoint)
                       { 
                         V_ref = V_Dsetpoint; //set final set point for inverter voltage
                           
                        }
              
               PO_mppt();    
                
       		                    ffd_Flag = 0;
       							}
//soft start end //

    			ClrWdt();
    		}//while end////////////////////////////////////////////////////////

  
		ClrWdt();
	} //main end////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////


//T1 interrupt for oscillator tracking
		void _ISRFAST __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) //99.88Khz
  			{   
                // harmonic oscillator to generate sin and cos ref

                if(osc_Flag)//7.5Khz
                 {
                  //harmonic oscillator
                  OSC_F = OSC_Fcentral;    // set inverter output voltage frequency
                 
                  theta = theta + OSC_F;   //update theta angle 
     
         		    if(theta >= theta_2PI) //reset oscillator
            		    {
           				qSin = 0;
           				qCos = 32440; //0.99
           				theta = 0;
              				}
              					else asmHARMONIC(); //harmonic oscillator
                    
                    //
                      osc_Flag = 0;
                      } 
                
     			T1us_Flag = 0;
                
   					} //T1 interupt end

///////////////////////////////////////////////////////////////////////

		//fault interrupt
		void _ISR __attribute__((interrupt, no_auto_psv)) _FLTBInterrupt(void)
  			 {
     			PWMenable = 0; //disable pwm if fault
     			SET = 0;       //all switches off
          
     			RL2_ON = 0;    //open load relay
                 
		fault_Flag = 0; 
            
   			}//fault end

//////////////////////////////////////////////////////////////////////

			//initial startup routine
			void starting(void)
  				{
                    PWM_offset = PWM_PERIOD; //initialise PWM period value
                    PWM_offset_N = -PWM_PERIOD;

					PWM_max = PWM_offset*8; //PI saturation values
					PWM_min = -PWM_offset*8;
					SET = 0; //initialise PWM control registers
					PWMenable = 0; //reset PWM control register
					 //
					FAULT_ENABLE = 1; //0x000f; //reset fault register
					delay(30); //delay 30ms
					ADC_ON = 1;

					//precharging init
					RL1_ON = 1;  //precharging enable
					delay(15); //delay 1500ms
					//precharging init ends
					
					offset = asmADC(0x0e0e); //2.5V offset //read adc channel 14

					//
					//initiates startup
					RL1_ON = 0;  //precharging disable
					delay(30); //delay 30ms
					RL2_ON = 1;  //bypass precharging
					delay(30); //delay 30ms
					
					//set pwm
					PWM1 = PWM_offset;
					PWM2 = PWM_offset;
					PWM3 = PWM_offset;
					//SET = 0x0077; //all three switces are enabled
					//

					PWMenable = 1; //enable PWM and timers
					T1ON = 1;
                    T2ON = 1;
                    T3ON = 1;
                    T4ON = 1;
                    T5ON = 1;
                    
					// 
					  	}//startup routine end

///////////////////////////////////////////////////////////////////////

void PO_mppt() //perturb and observe mppt algorithm
{
           asm("disi #0x3FFF");
           {
          Vpv = asmADC(0x0007);  // pv voltage //adc channel 7
          Ipv = asmADC(0x000A);  // pv current //adc channel 10    
            } 

	Pnew = asmINT_MPQ(Vpv,Ipv); //PV new power value

	if(Pnew > Pold) //if power is increasing
	           {
	               if(Vpv > Vpv_old) PVref++; //if PV voltage is increasing increase ref
	               if(Vpv < Vpv_old) PVref--; //if PV voltage is decreasing decrease ref
	               Pold = Pnew;               //old power
	               Vpv_old = Vpv;             //old voltage
	                     }
	if(Pnew < Pold) //if power is decreasing
	            {
	               if(Vpv > Vpv_old) PVref--; //if PV voltage is increasing decrease ref
	               if(Vpv < Vpv_old) PVref++; //if PV voltage is decreasing increase ref
	               Pold = Pnew;               //old power
	               Vpv_old = Vpv;             //old voltage
	                      }

	     if(PVref <= 0) PVref = 0;
	        if(PVref >= PVref_max) PVref = PVref_max; //set max mppt ref
}


			











