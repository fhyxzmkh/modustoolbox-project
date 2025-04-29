#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Define how many samples in a frame */
#define PDM_PCM_BUFFER_SIZE                 (224000u) // 16000 * 2 * 7 (7 seconds)

/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ                      16000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE                     64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ                  24576000u
/* PDM/PCM Pins */
#define PDM_DATA                            P10_5
#define PDM_CLK                             P10_4
/* Switch press/release check interval in milliseconds for debouncing */
#define SWITCH_DEBOUNCE_CHECK_UNIT          (1u)
/* Number of debounce check units to count before considering that switch is pressed or released */
#define SWITCH_DEBOUNCE_MAX_PERIOD_UNITS    (80u)

#define AMBIENT_TEMPERATURE_C               (20)
#define SPI_BAUD_RATE_HZ                    (20000000)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
void clock_init(void);
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
static uint32_t read_switch_status(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
volatile bool pdm_pcm_flag = false;

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_LEFT,
    .word_length     = 16,  /* bits */
    .left_gain       = CY_PDM_PCM_GAIN_10_5_DB,   /* dB */
    .right_gain      = 0U,   /* dB */
};


/* Pointer to the new frame that need to be written */
uint8_t *current_frame;



/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint8_t  audio_frame[PDM_PCM_BUFFER_SIZE] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
    clock_init();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);


    /* Initialize the User LEDs */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the User button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);

    /* GPIO initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Initialize the PDM/PCM block */
    cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);

    /* To avoid compiler warning */
    (void)result;

    /* Show the startup screen */
    //printf("Start\n");

    for(;;)
    {
        /* Check if any microphone has data to process */
        if (pdm_pcm_flag)
        {
        	//printf("\nWriting to UART...\n");
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

            /* Clear the PDM/PCM flag */
            pdm_pcm_flag = 0;


            printf("\nSTART\n");
            for (uint32_t index = 0; index < PDM_PCM_BUFFER_SIZE; index++)
            {
                cyhal_uart_putc(&cy_retarget_io_uart_obj, audio_frame[index]);
            }
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
            //printf("Press SW2\n");

        }
        if (0UL != read_switch_status())
        {
            //printf("Recording\n");

            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
            cyhal_system_delay_ms(1000);

            /* Read the next frame */
            cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, PDM_PCM_BUFFER_SIZE/2);
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        }

        cyhal_system_delay_ms(1);
    }
}


void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;

    pdm_pcm_flag = true;
}

void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_get(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_init(&pll_clock);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_get(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_init(&audio_clock);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

uint32_t read_switch_status(void)
{
    uint32_t delayCounter = 0;
    uint32_t sw_status = 0;

    /* Check if the switch is pressed */
    while(0UL == cyhal_gpio_read(CYBSP_USER_BTN))
    {
        /* Switch is pressed. Proceed for debouncing. */
        cyhal_system_delay_ms(SWITCH_DEBOUNCE_CHECK_UNIT);
        ++delayCounter;

        /* Keep checking the switch status till the switch is pressed for a
         * minimum period of SWITCH_DEBOUNCE_CHECK_UNIT x SWITCH_DEBOUNCE_MAX_PERIOD_UNITS */
        if (delayCounter > SWITCH_DEBOUNCE_MAX_PERIOD_UNITS)
        {
            /* Wait till the switch is released */
            while(0UL == cyhal_gpio_read(CYBSP_USER_BTN))
            {
            }

            /* Debounce when the switch is being released */
            do
            {
                delayCounter = 0;

                while(delayCounter < SWITCH_DEBOUNCE_MAX_PERIOD_UNITS)
                {
                    cyhal_system_delay_ms(SWITCH_DEBOUNCE_CHECK_UNIT);
                    ++delayCounter;
                }

            }while(0UL == cyhal_gpio_read(CYBSP_USER_BTN));

            /* Switch is pressed and released*/
            sw_status = 1u;
        }
    }

    return (sw_status);
}

