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
  
  - ### DAC3 (unbuffered) with OPAMP Min and Max transitions using DMA and TIM6

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
		
  ## FW setup TIM6
  
  - ### After ** MX_TIM6_Init()** by _CubeMX_ use below code codes to activate TIM6  
  
		LL_TIM_EnableCounter(TIM6);
		
		// Trigger at _Update_ _Event_  
		
		LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  
  	### Some issues in the FW https://github.com/STMicroelectronics/STM32CubeG4/issues/24  
	https://community.st.com/s/question/0D53W00000aqkBISAY/timer-prescaler-setting-math-operation-will-produce-wrong-code
	
  ## DAC
  
  - ### After ** MX_TIM6_Init()** by _CubeMX_ use below code codes to activate DAC3  
  
		void Activate_DAC(void)
		{
			__IO uint32_t wait_loop_index = 0;

			/* Enable DAC channel */
			LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);

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
			LL_DAC_EnableTrigger(DAC3, LL_DAC_CHANNEL_1);
		}
  
  ## DAC DMA
  
  - ### Setting up ** MX_DAC3_Init()** by _CubeMX_ DMA Peripheral Size shoudl be _Word_ _Length_  
  
		LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
		
  - ### Enable DMA by below code  
  
		void LL_DAC3_EnableDMA(void)
		{
			  /* Set DMA transfer addresses of source and destination */
			  LL_DMA_ConfigAddresses(DMA1,
									 LL_DMA_CHANNEL_1,
									 (uint32_t)&MinMaxsamples,
									 LL_DAC_DMA_GetRegAddr(DAC3, LL_DAC_CHANNEL_1, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED),
									 LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

			  /* Set DMA transfer size */
			  LL_DMA_SetDataLength(DMA1,
								   LL_DMA_CHANNEL_1,
								   MinMaxsamples_SIZE);

			  /* Enable DMA transfer interruption: transfer error */
		//	  LL_DMA_EnableIT_TE(DMA1,
		//	                     LL_DMA_CHANNEL_1);

			  /* Note: In this example, the only DMA interruption activated is            */
			  /*       transfer error.                                                     */
			  /*       If needed, DMA interruptions of half of transfer                   */
			  /*       and transfer complete can be activated.                            */
			  /*       Refer to DMA examples.                                             */

			  /* Activation of DMA */
			  /* Enable the DMA transfer */
			  LL_DMA_EnableChannel(DMA1,
								   LL_DMA_CHANNEL_1);

			  /* Set DAC mode sample-and-hold timings */
			  // LL_DAC_SetSampleAndHoldSampleTime (DAC1, LL_DAC_CHANNEL_1, 0x3FF);
			  // LL_DAC_SetSampleAndHoldHoldTime   (DAC1, LL_DAC_CHANNEL_1, 0x3FF);
			  // LL_DAC_SetSampleAndHoldRefreshTime(DAC1, LL_DAC_CHANNEL_1, 0xFF);

			  /* Set the mode for the selected DAC channel */
			  // LL_DAC_SetMode(DAC1, LL_DAC_CHANNEL_1, LL_DAC_MODE_NORMAL_OPERATION);

			  /* Enable DAC channel DMA request */
			  LL_DAC_EnableDMAReq(DAC3, LL_DAC_CHANNEL_1);

			  /* Enable interruption DAC channel1 under-run */
		//	  LL_DAC_EnableIT_DMAUDR1(DAC1);
		}
