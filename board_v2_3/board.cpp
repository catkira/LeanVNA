#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/flash.h>
#include <mculib/fastwiring.hpp>
#include <mculib/softi2c.hpp>
#include <mculib/softspi.hpp>
#include <mculib/si5351.hpp>
#include <mculib/adf4350.hpp>
#include <mculib/dma_adc.hpp>

#include <array>
#include <stdint.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/adc.h>

#include "board.hpp"
#include "../rfsw.hpp"

using namespace mculib;
using namespace std;

namespace board {

	// set by board_init()
	uint32_t hseEstimateHz = 0;
	uint32_t adc_ratecfg = 0;
	uint32_t adc_srate = 0; // Hz
	uint32_t adc_period_cycles, adc_clk;


	DMADriver dma(DMA1);
	DMAChannel dmaChannelADC(dma, 1);
	DMAChannel dmaChannelSPI(dma, 3);
	DMAADC dmaADC(dmaChannelADC, ADC1);


	i2cDelay_t i2cDelay;
	spiDelay_t spiDelay;

	// TODO(gabu-chan): get rid of template argument after switch to c++17
	SoftI2C<i2cDelay_t> si5351_i2c(i2cDelay);
	Si5351::Si5351Driver si5351;

	SoftSPI<spiDelay_t> adf4350_tx_spi(spiDelay);
	SoftSPI<spiDelay_t> adf4350_rx_spi(spiDelay);


	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_tx(adf4350_sendWord_t {adf4350_tx_spi});
	ADF4350::ADF4350Driver<adf4350_sendWord_t> adf4350_rx(adf4350_sendWord_t {adf4350_rx_spi});

	
	// same as rcc_set_usbpre, but with extended divider range:
	// 0: divide by 1.5
	// 1: divide by 1
	// 2: divide by 2.5
	// 3: divide by 2
	void rcc_set_usbpre_gd32(uint32_t usbpre) {
		uint32_t RCC_CFGR_USBPRE_MASK = uint32_t(0b11) << 22;
		uint32_t old = (RCC_CFGR & ~RCC_CFGR_USBPRE_MASK);
		RCC_CFGR = old | ((usbpre & 0b11) << 22);
	}

	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV2 = 0b0000;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV4 = 0b0001;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV6 = 0b0010;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV8 = 0b0011;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV12 = 0b0101;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16 = 0b0111;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV5 = 0b1000;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV6 = 0b1001;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV10 = 0b1010;
	constexpr uint32_t GD32_RCC_CFGR_ADCPRE_HCLK_DIV20 = 0b1011;

	static void rcc_set_adcpre_gd32(uint32_t adcpre) {
		uint32_t RCC_CFGR_ADCPRE_MASK = (0b11 << 14) | (1 << 28);
		uint32_t RCC_CFGR2_ADCPRE_MASK = 1 << 29;
		uint32_t old = (RCC_CFGR & ~RCC_CFGR_ADCPRE_MASK);
		uint32_t old2 = (RCC_CFGR2 & ~RCC_CFGR2_ADCPRE_MASK);
		RCC_CFGR = old | ((adcpre & 0b11) << 14) | ((adcpre & 0b100) << (28 - 2));
		RCC_CFGR2 = old2 | ((adcpre & 0b1000) << (29 - 3));
	}

	/* TODO this code is a copy of code in libopencm3. 
	 * Should this move to libopencm3? */
	void rcc_clock_setup_in_hse_24mhz_out_96mhz(void) {
		 /* Enable internal high-speed oscillator. */
		 rcc_osc_on(RCC_HSI);
		 rcc_wait_for_osc_ready(RCC_HSI);

		 /* Select HSI as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

		 /* Enable external high-speed oscillator. */
		 rcc_osc_on(RCC_HSE);
		 rcc_wait_for_osc_ready(RCC_HSE);
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

		 /*
		  * Set prescalers for AHB, ADC, ABP1, ABP2.
		  * Do this before touching the PLL (TODO: why?).
		  */
		 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 96MHz Max. 96MHz
		 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_PCLK2_DIV16);		// Set. 6MHz Max. 40MHz
		 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 48MHz Max. 60MHz
		 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 96MHz Max. 120MHz
		 rcc_set_usbpre_gd32(3);									// 96MHz / 2 = 48MHz

		 /*
		  * Sysclk runs with 96MHz -> 0 waitstates.
		  */
		 flash_set_ws(FLASH_ACR_LATENCY_0WS);

		 /*
		  * Set the PLL multiplication factor. (x4) 24 * 4 = 96Mhz
		  */
		 rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL4);

		 /* Select HSE as PLL source. */
		 rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

		 /*
		  * External frequency undivided before entering PLL
		  * (only valid/needed for HSE).
		  */
		 rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

		 /* Enable PLL oscillator and wait for it to stabilize. */
		 rcc_osc_on(RCC_PLL);
		 rcc_wait_for_osc_ready(RCC_PLL);

		 /* Select PLL as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

		 /* Set the peripheral clock frequencies used */
		 rcc_ahb_frequency = 96000000;
		 rcc_apb1_frequency = 48000000;
		 rcc_apb2_frequency = 96000000;

		 cpu_mhz = 96;
	}

	void rcc_clock_setup_in_hse_24mhz_out_120mhz(void) {
		 cpu_mhz = 120;
		 // only run the initialization once
		 if(RCC_CR & RCC_CR_HSEON) return;

		 /* Enable internal high-speed oscillator. */
		 rcc_osc_on(RCC_HSI);
		 rcc_wait_for_osc_ready(RCC_HSI);

		 /* Enable external high-speed oscillator. */
		 rcc_osc_on(RCC_HSE);
		 rcc_wait_for_osc_ready(RCC_HSE);

		 /* Select HSE as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);
		 
		 rcc_osc_off(RCC_PLL);
		 /*
		  * Set prescalers for AHB, ADC, ABP1, ABP2.
		  * Do this before touching the PLL (TODO: why?).
		  */
		 rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);					// Set. 120MHz Max. 120MHz
		 rcc_set_adcpre_gd32(GD32_RCC_CFGR_ADCPRE_PCLK2_DIV4);		// Set. 30MHz Max. 40MHz
		 rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);					// Set. 60MHz Max. 60MHz
		 rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);					// Set. 120MHz Max. 120MHz
		 rcc_set_usbpre_gd32(2);									// 120MHz / 2.5 = 48MHz

		 /*
		  * Sysclk runs with 120MHz -> 0 waitstates.
		  */
		 flash_set_ws(FLASH_ACR_LATENCY_0WS);

		 /*
		  * Set the PLL multiplication factor. (x5) 24 * 5 = 120Mhz
		  */
		 rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL5);

		 /* Select HSE as PLL source. */
		 rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

		 /*
		  * External frequency undivided before entering PLL
		  * (only valid/needed for HSE).
		  */
		 rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

		 /* Enable PLL oscillator and wait for it to stabilize. */
		 rcc_osc_on(RCC_PLL);
		 rcc_wait_for_osc_ready(RCC_PLL);

		 /* Select PLL as SYSCLK source. */
		 rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

		 /* Set the peripheral clock frequencies used */
		 rcc_ahb_frequency = 120000000;
		 rcc_apb1_frequency = 60000000;
		 rcc_apb2_frequency = 120000000;
	}

	void boardInit() {
		rcc_clock_setup_in_hse_24mhz_out_120mhz();

		// enable basic peripherals
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_GPIOC);

		rcc_periph_clock_enable(RCC_AFIO);
		// jtag pins should be used as GPIOs (SWD is used for debugging)
		gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
		
		si5351_i2c.clk = PB1;
		si5351_i2c.sda = PB0;

		adf4350_tx_spi.sel = PA1;
		adf4350_tx_spi.clk = PA2;
		adf4350_tx_spi.mosi = PA3;
		adf4350_tx_spi.miso = PB10; // unused

		adf4350_rx_spi.sel = PA4;
		adf4350_rx_spi.clk = PA2;
		adf4350_rx_spi.mosi = PA3;
		adf4350_rx_spi.miso = PB10;

		adf4350_tx_spi.init();
		adf4350_rx_spi.init();

		adc_ratecfg = ADC_SMPR_SMP_7DOT5CYC;
		adc_srate = 30000000/(7.5+12.5); // 1.5 MSPS
		adc_period_cycles = (7.5+12.5);
		adc_clk = 30000000;
	}


	// returns an estimate of the HSE frequency in Hz.
	// called by boardInit() to set hseEstimateHz.
	uint32_t detectHSEFreq() {
		rcc_osc_on(RCC_HSE);
		rcc_osc_on(RCC_HSI);
		rtc_auto_awake(RCC_HSE, 1 << 19);
		rtc_exit_config_mode();
		delay(2);
		uint32_t tmp = rtc_get_prescale_div_val();
		delay(20);
		uint32_t tmp2 = rtc_get_prescale_div_val();
		// cycles of a fHSE/128 clock elapsed
		uint32_t cycles = (tmp - tmp2) & ((1 << 20) - 1);
		uint32_t freqHz = cycles * 128 * 50;
		return freqHz;
	}

	void ledPulse() {
		digitalWrite(led2, HIGH);
		delayMicroseconds(1);
		digitalWrite(led2, LOW);
	}

	int calculateSynthWaitAF(freqHz_t freqHz) {
		if(freqHz < 3100000000) return 10;
		return 24;
	}

	int calculateSynthWaitSI(int retval) {
		switch(retval) {
			case 0: return 36;
			case 1: return 120;
			case 2: return 32;
		}
		return 5;
	}

}
