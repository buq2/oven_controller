#include <avr/io.h>
#include <avr/interrupt.h>
#include <LUFA/Platform/Platform.h>
#include <LUFA/Drivers/Peripheral/TWI.h>
#include <LUFA/Drivers/Peripheral/SPI.h>
#include <LUFA/Drivers/USB/USB.h>
#include "lufa_config/Descriptors.h"
#include "thermo_max6675.hh"
#include <ctype.h>
#include <stdlib.h>
#include "xmega-libraries/core/pwm.hh"
#include "xmega-libraries/control/pid_controller.hh"

// If we ever run pure virtual funciton, stop
extern "C" void __cxa_pure_virtual() { while (1); }

#define STOP_IF_ERROR(x) {if(x){while(1){}}}

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

void EVENT_USB_Device_Connect(void)
{

}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{

}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
    /* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
    XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
    XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

    /* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
    XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
    XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

    PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

    /* Hardware Initialization */
    USB_Init();
}

int main()
{
    SetupHardware();

    /* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
    GlobalInterruptEnable();

    ThermoMax6675 thermo(axlib::PORT_C, axlib::PORT_C, 0b00001000);
    thermo.SetTemperatureBiasCelcius(4);
    axlib::PwmSimple pwm(axlib::PORT_D, 0b00000001);
    pwm.SetClock(TC_CLKSEL_DIV256_gc);
    pwm.SetPeriod(0xFFFF);
    pwm.SetDutyCycle(0);
    axlib::PidController<8> pid;
    pid.SetI(0.001);
    pid.SetControlChangeMinMax(-0.5,0.1);
    pid.SetP(0.002);
    pid.SetD(-0.003);
    pid.SetDeadZone(3);

    char str[128];

    char setpoint_str[32];
    uint8_t setpoint_str_pos = 0;
    while (1) {
        // Try to get set point
        int16_t inchar = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
        if (inchar > 0) {
            char c = inchar;
            if (isdigit(c) && setpoint_str_pos < 31) {
                setpoint_str[setpoint_str_pos] = c;
                ++setpoint_str_pos;
            } else {
                setpoint_str[setpoint_str_pos] = '\0';
                setpoint_str_pos = 0;

                // Try to convert
                int val = atoi(setpoint_str);
                if (c == 'i') {
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Integral gain changed to: ");
                    const float i = ((float)val)*0.0001;
                    pid.SetI(i);
                    sprintf(str,"%f", (double)i);
                } else if (c == 'p') {
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Proportional gain changed to: ");
                    const float p = ((float)val)*0.0001;
                    pid.SetP(p);
                    sprintf(str,"%f", (double)p);
                } else if (c == 'd') {
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Derivative gain changed to: ");
                    const float d = -((float)val)*0.001;
                    pid.SetD(d);
                    sprintf(str,"%f", (double)d);
                } else if (c == 'e') {
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Deadzone changed to: ");
                    const float d = val;
                    pid.SetDeadZone(d);
                    sprintf(str,"%f", (double)d);
                } else {
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Set point changed to: ");
                    sprintf(str,"%d", val);
                    pid.SetSetPoint(val);
                }
                CDC_Device_SendString(&VirtualSerial_CDC_Interface, str);
                CDC_Device_SendString(&VirtualSerial_CDC_Interface, "\n\r");
            }
        }

        uint16_t temp = 0;
        thermo.GetTemperatureCelcius(&temp);
        pid.Update(temp);

        float control = pwm.GetDutyCycle();;
        bool success = pid.ModifyControl(&control);
        control = axlib::min(axlib::max(control, 0.0f), 1.0f);
        if (success) {
            sprintf(str,"Temp: %d, Set: %d, DutyCycle: %f\n\r", temp, (uint16_t)pid.GetSetPoint(), (double)control);
            pwm.SetDutyCycle(control);
        } else {
            sprintf(str,"Temp: %d\n\r", temp);
        }

        CDC_Device_SendString(&VirtualSerial_CDC_Interface, str);

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
        _delay_ms(250);
    }
}
