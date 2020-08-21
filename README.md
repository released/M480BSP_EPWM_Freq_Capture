# M480BSP_EPWM_Freq_Capture
 M480BSP_EPWM_Freq_Capture

update @ 2020/08/21

1. use EPWM1_CH5 (PB.6) with PDMA , to capture signal freq input : EPWM0_CH2 (PA.3)

2. use URART terminal to switch EPWM0_CH2 (PA.3) duty , by press 1 (increase) or 2 (decrease)

3. Connect output : EPWM0_CH2 (PA.3) to input : EPWM1_CH5 (PB.6) , to simulate external signal

4. how to calculate ns ? 

	- (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
	
	- how to get PSC , make CNR+1 close to 0xFFFF , target freq = 15
	
	- CNR+1 = 192000000 / PSC / min_freq , when CNR = 65535 with min freq = 10 , will get PSC 293
	
	- Capture unit time = 1/Capture clock source frequency/prescaler
	
	- target ns = 1/ 192000000 / 293 = 293/ 192000000 = 1526 ns

5. below is capture screen ,

duty : 10 percent

![image](https://github.com/released/M480BSP_EPWM_ECAP/blob/master/scope_duty_10.jpg)

![image](https://github.com/released/M480BSP_EPWM_ECAP/blob/master/log_duty_10.jpg)

duty : 90 percent

![image](https://github.com/released/M480BSP_EPWM_ECAP/blob/master/scope_duty_90.jpg)

![image](https://github.com/released/M480BSP_EPWM_ECAP/blob/master/log_duty_90.jpg)




