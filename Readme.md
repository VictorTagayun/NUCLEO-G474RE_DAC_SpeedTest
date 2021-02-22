## Speed Test of DAC on different configurations  

Study the speed of DAC using buffered, unbuffered and together with OPAMP follower

  - ### DAC1 Buffered and Unbuffered 160Mhz  
  
		Unbuffereduffered  
		![]() 
		
		Buffered  
		![]() 
  
  - ### DAC3 (unbuffered) with OPAMP
  
		![]()
		
		
  - ### OPAMP Speed used as follower
  
		![]()
  
  - ### DAC3 (unbuffered) with OPAMP Min and Max transitions using DMA and HRTIM

		![]()
		
		
  ## FW setup LL DAC  
  
  - ### After **MX_DAC1_Init()** by _CubeMX_ use below codes to activate DAC  
  
		void Activate_DAC(void)
		{
			__IO uint32_t wait_loop_index = 0;

			/* Enable DAC channel */
			LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

			/* Delay for DAC channel voltage settling time from DAC channel startup.    */
			/* Compute number of CPU cycles to wait for, from delay in us.              */
			/* Note: Variable divided by 2 to compensate partially                      */
			/*       CPU processing cycles (depends on compilation optimization).       */
			/* Note: If system core clock frequency is below 200kHz, wait time          */
			/*       is only a few CPU processing cycles.                               */
			wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US * (SystemCoreClock / (100000 * 2))) / 10);
			while(wait_loop_index != 0)
			{
				wait_loop_index--;
			}

			/* Enable DAC channel trigger */
			/* Note: DAC channel conversion can start from trigger enable:              */
			/*       - if DAC channel trigger source is set to SW:                      */
			/*         DAC channel conversion will start after trig order               */
			/*         using function "LL_DAC_TrigSWConversion()".                      */
			/*       - if DAC channel trigger source is set to external trigger         */
			/*         (timer, ...):                                                    */
			/*         DAC channel conversion can start immediately                     */
			/*         (after next trig order from external trigger)                    */
			LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
		}
		
 - ### To change value of DAC and out the value, use the ff codes:  
		/* Set the data to be loaded in the data holding register */
		LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, tmp_dac_value);

		/* Trig DAC conversion by software */
		LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
		
  ## FW setup OPAMP
  
  - ### After ** MX_OPAMP6_Init()** by _CubeMX_ use below code codes to activate OPAMP  
  
		void Activate_OPAMP(void)
		{
			__IO uint32_t wait_loop_index = 0;

			/* Enable OPAMP */
			LL_OPAMP_Enable(OPAMP2);

			/* Delay for OPAMP startup time.                                            */
			/* Compute number of CPU cycles to wait for, from delay in us.              */
			/* Note: Variable divided by 2 to compensate partially                      */
			/*       CPU processing cycles (depends on compilation optimization).       */
			/* Note: If system core clock frequency is below 200kHz, wait time          */
			/*       is only a few CPU processing cycles.                               */
			wait_loop_index = ((LL_OPAMP_DELAY_STARTUP_US * (SystemCoreClock / (100000 * 2))) / 10);
			while(wait_loop_index != 0)
			{
				wait_loop_index--;
			}
		}
		
  ## FW setup HRTIM, DMA and DAC
  
  