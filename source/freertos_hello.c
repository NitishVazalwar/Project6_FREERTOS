/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <math.h>
#include "CircBuffer.h"
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "math_functions.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dac.h"
#include "fsl_adc16.h"
#include "fsl_dma.h"
#include "board.h"
#include "lookup.h"
#include "pin_mux.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_DAC_BASEADDR DAC0

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /* PTE20, ADC0_SE0 */
#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define VREF_BRD 3.300
#define SE_12BIT 4096.0

 uint16_t buffer[50];
 uint16_t destaddr[64]={0};

volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue = 0;
adc16_channel_config_t g_adc16ChannelConfigStruct;

/*******************************************************************************
 * SW_Timer
 ******************************************************************************/
const TickType_t xDelay100ms = pdMS_TO_TICKS( 100 );
static void SwTimerCallback(TimerHandle_t xTimer);
void DMA_Callback(dma_handle_t *handle, void *param);

/* Task priorities. */
#define dac_task_PRIORITY (configMAX_PRIORITIES - 3)
#define adc_task_PRIORITY (configMAX_PRIORITIES-2)
#define dsp_task_PRIORITY (configMAX_PRIORITIES-1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DACTask(void *pvParameters);
static void ADCTask(void *pvParameters);
static void DSPTask(void *pvParamaters);
/*******************************************************************************
 * Declarations
 ******************************************************************************/
CircBuffer_t * Buffer;
  uint8_t flag_w=0;
  uint8_t flag_dsp=0;
  dma_handle_t g_DMA_Handle;
  volatile bool g_Transfer_Done = false;
#define DMA_CHANNEL 0
#define DMA_SOURCE 63
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    TimerHandle_t SwTimerHandle = NULL;
      SystemCoreClockUpdate();
      /* Create the software timer. */
      SwTimerHandle = xTimerCreate("SwTimer",          /* Text name. */
                                   xDelay100ms, /* Timer period. */
                                   pdTRUE,             /* Enable auto reload. */
                                   0,                  /* ID is not used. */
                                   SwTimerCallback);   /* The callback function. */
      /* Start timer. */
      xTimerStart(SwTimerHandle, 0);
    xTaskCreate(DACTask, "DACTask", configMINIMAL_STACK_SIZE+100, NULL, dac_task_PRIORITY, NULL);
    xTaskCreate(ADCTask, "ADCTask", configMINIMAL_STACK_SIZE+100, NULL, adc_task_PRIORITY, NULL);
    vTaskStartScheduler();
    for (;;)
        ;
}


void DMA_Callback(dma_handle_t *handle, void *param)
{
    g_Transfer_Done = true;
}


static void ADCTask(void *pvParameters)
{

	Buffer=CircBufferCreate();
	CBufferReturn_t ret 	= CircularBufferInit(Buffer, SIZE);
	if(ret != SUCCESS)
				{
					printf("Circular buffer  failed");

					//logString(LL_Debug, FN_uartInit, "Creation of rx Buffer Failed\n\r\0");
					return;
				}

	//ADC init starts
	TickType_t Lastwakeuptime;
	Lastwakeuptime = xTaskGetTickCount();
	printf("\n\rADC TASk");
	 adc16_config_t adc16ConfigStruct;
	 adc16_channel_config_t adc16ChannelConfigStruct;

	 ADC16_GetDefaultConfig(&adc16ConfigStruct);
	#if defined(BOARD_ADC_USE_ALT_VREF)
	    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif
	    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

	    /* Make sure the software trigger is used. */
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
	    {
	        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
	    }
	    else
	    {
	        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	    /* Prepare ADC channel setting */
	    g_adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
	//End of ADC initialisation


	    void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
	    {
	        g_Adc16ConversionDoneFlag = true;
	        /* Read conversion result to clear the conversion completed flag. */
	        g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
	    }



	    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	    			        while (0U == (kADC16_ChannelConversionDoneFlag &
	    			                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP)))
	    			        {
	    			        }

	    			        PRINTF("ADC Value: %d\r\n", ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP));
	    			        CBAdd(Buffer, g_Adc16ConversionValue, &flag_dsp);

	    			        DMAMUX_Init(DMAMUX0);
	    			            DMAMUX_SetSource(DMAMUX0, DMA_CHANNEL, DMA_SOURCE);
	    			            DMAMUX_EnableChannel(DMAMUX0, DMA_CHANNEL);
	    			            /* Configure DMA one shot transfer */
	    			            DMA_Init(DMA0);
	    			            DMA_CreateHandle(&g_DMA_Handle, DMA0, DMA_CHANNEL);
	    			            DMA_SetCallback(&g_DMA_Handle, DMA_Callback, NULL);
	    			        //    DMA_PrepareTransfer(&transferConfig, srcAddr, sizeof(srcAddr[0]), destaddr, sizeof(destaddr[0]), sizeof(srcAddr),
	    			       //                         kDMA_MemoryToMemory);
	    			        //    DMA_SubmitTransfer(&g_DMA_Handle, &transferConfig, kDMA_EnableInterrupt);
	    			            DMA_StartTransfer(&g_DMA_Handle);


	          if (flag_dsp==0){
	        	  xTaskCreate(DSPTask, "DSPTask", configMINIMAL_STACK_SIZE+100, NULL, dsp_task_PRIORITY, NULL);
	          }
	          vTaskDelayUntil(&Lastwakeuptime, xDelay100ms);
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void DACTask(void *pvParameters)
{

    for (;;)
    {
        PRINTF("Program one Dac-adc\r\n");


        TickType_t Lastwakeuptime;
        Lastwakeuptime = xTaskGetTickCount();


        //DAC
        dac_config_t dacConfigStruct;
        DAC_GetDefaultConfig(&dacConfigStruct);
            DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
            DAC_Enable(DEMO_DAC_BASEADDR, true);             /* Enable output. */
            DAC_SetBufferReadPointer(DEMO_DAC_BASEADDR, 0U); /* Make sure the read pointer to the start. */
                                                             /*
                                                             * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
                                                             * and items can be written manually by user.
                                                             */
            uint8_t index=0;
            //log values
//            for(index= 0;index<50;index++)
//               {
//               	DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, lookup[index]);
//               	PRINTF("DAC out: %d\r\n",lookup[index]);
//               }
            while(index<50)
            {
            	//DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, lookup[index]);
            	PRINTF("\n\rlookup index value is %d : %d ",lookup[index],index);
            	index++;
//            	if(50==index)
//            	{
//            		index=0;
//            	}

            	vTaskDelayUntil(&Lastwakeuptime, xDelay100ms);
            }

            //Note:Values are printed every 0.1s
            /*
             *  Voltage is determined as V=2+sin(2pi*t/5)
             */

            uint16_t val;
            float voltage=0;
            float step=0;
            for(int i=0;i<50;i++)
            		{
            		voltage=(float)2+sin((float)0.4*PI*step);
            		val = (uint16_t) ((SE_12BIT * voltage / VREF_BRD) -1);
            		buffer[i]=val;
            		step+=0.1;
            		}
            for(int i=0;i<50;i++)
            {
            	//TODO:use logger in the end
            	PRINTF("\n\r%d is buffer[%d] value",buffer[i],i);
            	DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, buffer[i]);
            	if(flag_w==1)
            	{
            		PRINTF("\n\r Value is set to DAC0 J10 pin");
            		DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, buffer[i]);
            		flag_w=0;
            	}

            }
    }
}

static void SwTimerCallback(TimerHandle_t xTimer)
{
	 for (int j=0;j<50;j++)
	    {
		 	 //note DAC0DAT is 16 bit buffer is uint32_t
		flag_w=1;
	    }
}

static void DSPTask(void *pvParamaters)
{
	PRINTF("\n\r DSP Task");
	float average=0;
	float minimum=0;
	float maximum =0;
	float variance;
	float standard_deviation;
	  uint8_t counter=0;

	  for (int i=0;i<64;i++)
	  {
		  if(destaddr[i]<minimum){
			  minimum=destaddr[i];
		  }

		  if(destaddr[i]>maximum){
		  			  maximum=destaddr[i];
		  		  }
		  average += destaddr[i];

	  }

	  average /=64;

	  variance= GetVariance(destaddr, 64, average);
	  standard_deviation= sqrtf(variance);



	  PRINTF("\n\rAverage : %f",average);
	  PRINTF("\n\rVariance : %f",variance);
	  PRINTF("\n\rStandard deviation : %f",standard_deviation);
	  counter++;

	  if(counter==5){
		  vTaskSuspendAll();
	  }

	  vTaskSuspend(NULL);

}
