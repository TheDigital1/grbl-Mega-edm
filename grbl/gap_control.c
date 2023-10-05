/*
  gap_control.c - gap voltage control 
  Part of Grbl for EDM
  Author: The_Digital_1
*/

#include "grbl.h"

void InitADC()
{
    // Select Vref=AVcc
    ADMUX |= (1 << REFS0);
    //set prescaller to 128 and enable ADC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
}
uint16_t ReadADC(uint8_t ADCchannel)
{
    //printFloat(gc_state.spindle_speed,3);
    //printString("ss\r\n");
    //select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    //single conversion mode
    ADCSRA |= (1 << ADSC);
    // wait until ADC conversion is complete
    while (ADCSRA & (1 << ADSC))
        ;
    return ADC;
}
static void report_util_axis_values(float *axis_value)
{
    uint8_t idx;
    for (idx = 0; idx < N_AXIS; idx++)
    {
        printFloat_CoordValue(axis_value[idx]);
        if (idx < (N_AXIS - 1))
        {
            serial_write(',');
        }
    }
}

void gap_init()
{
    //Dont do this it screws up the interrupts on this port. Seems to work fine without it anyways.
    GAP_CONTROL_DDR &= ~(bit(GAP_ANALOG_INPUT_PIN)); // Configure as input pin
    GAP_CONTROL_PORT &= ~(bit(GAP_ANALOG_INPUT_PIN)); // Normally a voltage divider input. Disable pullup as to not affect voltage reading.
    //DDRF |= bit(4); // JTAG Clock is A4, noise on A3 can get into it causeing issues. Setting this Pin to an output and driving it low should help. ** I was wrong this was not the issue
    //PORTF &= ~(bit(4));
    InitADC();
    gap_flags = 0;
    //Setup timer5 to be the cycle timer for the gap_control

    // Configure Timer 5:
    // NOTE: By using an overflow interrupt, the timer is automatically reloaded upon overflow.
    TCCR5B = 0; // Normal operation. Overflow.
    TCCR5A = 0;
    TCCR5B = (TCCR5B & ~((1 << CS52) | (1 << CS51))) | (1 << CS50); // Stop timer
    // TCCR5B |= (1<<CS52); // Enable timer with 1/256 prescaler. ~4.4min max with uint8 and 1.05sec/tick
    // TCCR5B |= (1<<CS51); // Enable timer with 1/8 prescaler. ~8.3sec max with uint8 and 32.7msec/tick
    TCCR5B |= (1 << CS51) | (1 << CS50); // Enable timer with 1/64 prescaler. ~66.8sec max with uint8 and 0.262sec/tick
    // TCCR3B |= (1<<CS32)|(1<<CS30); // Enable timer with 1/1024 prescaler. ~17.8min max with uint8 and 4.19sec/tick
    TIMSK5 |= (1 << TOIE5); // Enable Timer5 OVF Interrupt

    memset(path_history, 0, sizeof(path_history));
    path_history_depth = 0;
    path_history_pos = 0;
}

/*
The heart of gap control. Measures voltage of gap and adjusts feedrate.
If gap voltage is less than setpoint the feedrate is slowed down.
If gap voltage is greater than setpoint the feedrate is sped up.
If gap voltage is lower than min setpoint a gap recovery will be triggered, which will attempt to back up in the path.
Gap control can be enabled or disabled using the coolent flood toggle. CMD_COOLANT_FLOOD_OVR_TOGGLE 0xA0
Setpoint is controled by the tool variable. (0 - 255), Vref for the ADC is 5v so step size is about 20mv or (5/255)
Gap setpoint can be adjusted using CMD_FEED_OVR_FINE_PLUS (0x9C), and CMD_FEED_OVR_FINE_MINUS (0x9D) each corresponds to 1 tick up or down.
Todo: Make a specfic gap control flag. Make a specfic Gcode for gap voltage
*/
void gap_update_feedrate()
{
    //printFreeMemory();
    //printString("sys_state: ");
    //printInteger(DDRF);
    //printString("\r\n"); 

    sys_gap_read_state = false;
    //if (gc_state.modal.coolant)

    //sys_gap_read_state = false;
    //system_convert_array_steps_to_mpos(backwards_target,sys_position);

    uint16_t adc_val = 0;

    adc_val = ReadADC(3);
    float gap_voltage = 0.00492 * (float)adc_val; //This was cal'd with a DMM, Will likely be different for different boards
    static float old_gap_voltage = 1.5;           //1.5 is a decent starting point
    ave_gap_voltage = 0;

    //Exponential moving average... sort of
    ave_gap_voltage = gap_adc_ave_alpha * gap_voltage + (1 - gap_adc_ave_alpha) * old_gap_voltage;
    old_gap_voltage = ave_gap_voltage;

    if (coolant_get_state() & COOLANT_STATE_MIST)
    {
        //Check to see if there is a short to the workpiece. If so attempt to recover
        gap_clear_short(ave_gap_voltage);
        //printFloat(ave_gap_voltage,3);
        //printString("\r\n");
    }
#ifdef GAP_VOLTAGE_CONTROL
    if (coolant_get_state() & COOLANT_STATE_FLOOD)
    {
        float gap_setpoint = gc_state.tool;
        const uint8_t gap_f_ovr_step = 1;
        static uint8_t gap_new_override = 100;
        //printFloat(gap_setpoint,3);
        //printPgmString(PSTR("\r\n"));
        if (!(gap_flags && GAP_FLAG_SHORT_RECOVERY_ACTIVE)) //Make sure we are not in gap recovery mode
        {
            float gap_error = (ave_gap_voltage - gap_setpoint) * 100;

            //Speed up
            if (gap_error > 0)
            {
                gap_new_override = gap_new_override + gap_f_ovr_step;
            }
            //Slow down
            if (gap_error < 0)
            {
                gap_new_override = (gap_new_override - (5 * gap_f_ovr_step));
            }

            //Limit the output
            if (gap_new_override > 200)
            {
                gap_new_override = 200;
            }
            if (gap_new_override < 10)
            {
                gap_new_override = 10;
            }
            //sys.f_override = gap_new_override;

            if (gap_new_override != sys.f_override)
            {
                sys.f_override = gap_new_override;
                sys.r_override = gap_new_override;
                sys.report_ovr_counter = 0; // Set to report change immediately
                plan_update_velocity_profile_parameters();
                plan_cycle_reinitialize();
            }
            /* 
                //Debug prints
                printFloat(gap_voltage,3);
                printString(",");
                printInteger(gap_new_override); 
                printString(",");
                printFloat(gap_setpoint,3);
                printPgmString(PSTR("\r\n")); */
            //printPgmString(PSTR("tick\r\n")); */
            //printInteger(gap_control_output);
            //printString("ovr\r\n");
        }
    }

/*         else //in a short condition set feed override back to 100
        {
            if (gap_new_override > 100)
            {
                sys.f_override = 100;
                sys.r_override = 100;
                sys.report_ovr_counter = 0; // Set to report change immediately
                plan_update_velocity_profile_parameters();
                plan_cycle_reinitialize();
            }
        } */
#endif
}

void gap_add_pos_to_history()
{

    //push history stack back one
    path_history_depth++;
    if (path_history_depth > backwards_buffer_size)
    {
        path_history_depth = backwards_buffer_size; //Cap out
    }
    uint8_t i;
    for (i = (path_history_depth - 1); i > 0; i--)
    {
        memcpy(path_history[i], path_history[i - 1], sizeof(path_history[i - 1]));
    }
    /*float print_position[N_AXIS];
     printString("Adding: ");
    system_convert_array_steps_to_mpos(print_position, sys_position);
    report_util_axis_values(print_position);
    printString("\r\n"); */
    memcpy(path_history[0], sys_position, sizeof(sys_position)); //Put current position at the top of the history
}
//This routine backs up in the path attempting to clear a short condition
void gap_clear_short(float gap_voltage)
{

    //Not currently in gap short recovery mode, not moving, and not idle
    if (!(gap_flags & GAP_FLAG_SHORT_RECOVERY_ACTIVE) && !(gap_flags & GAP_FLAG_Moving) && (sys.state != STATE_IDLE))
    {
        if (gap_voltage < min_gap_v_before_clear) //Short condition
        {
            //Immediately pause
            bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
            bit_true(gap_flags, GAP_FLAG_Moving); //This is needed as the protocol_exec_rt_system can cause this whole funtion to be called again. Need to track if we are still in a move op

            do
            {
                protocol_exec_rt_system();
                //protocol_execute_realtime();
                if (sys.abort)
                {
                    return;
                }
            } while (sys.suspend != SUSPEND_HOLD_COMPLETE);

            bit_false(gap_flags, GAP_FLAG_Moving);

            gap_add_pos_to_history();

            float print_position[N_AXIS];
            system_convert_array_steps_to_mpos(print_position, sys_position);
            printString("Short at:");
            report_util_axis_values(print_position);
            printString("\r\n");
            bit_true(gap_flags, GAP_FLAG_SHORT_RECOVERY_ACTIVE);
            plan_block_t *block = plan_get_current_block();
            if (block == NULL)
            {
                planer_restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
            }
            else
            {
                planer_restore_condition = (block->condition & PL_COND_SPINDLE_MASK) | coolant_get_state();
            }
            planer_restore_spindle_speed = block->spindle_speed;
        }
    }
    //Gap short recovery active
    if (gap_flags & GAP_FLAG_SHORT_RECOVERY_ACTIVE)
    {
        //Make sure motion is completed, as we are going to start some motion
        if (!(sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) && !(gap_flags & GAP_FLAG_Moving))
        {

            plan_line_data_t plan_data;
            plan_line_data_t *pl_data = &plan_data;
            memset(pl_data, 0, sizeof(plan_line_data_t));
            //Short cleared
            if ((gap_voltage > min_gap_v_before_clear) && !(gap_flags & GAP_FLAG_SHORT_RECOVERY_RESTORING))
            {
                printString("Short Cleared\r\n");
                /*                 if (path_history_pos != 0)
                {
                    path_history_pos--;
                } */
                bit_true(gap_flags, GAP_FLAG_SHORT_RECOVERY_RESTORING); //Set flag to indicate we need to start restoring
            }

            if (gap_voltage < min_gap_v_before_clear) //Still shorted
            {
                path_history_pos++;
                if (path_history_pos > backwards_buffer_size)
                {
                    path_history_pos = backwards_buffer_size;
                }                                          //cap out
                pl_data->feed_rate = PARKING_PULLOUT_RATE; //Usually the faster rate
            }
            //No short, stepping back to restore point
            if ((gap_voltage > min_gap_v_before_clear) && (gap_flags & GAP_FLAG_SHORT_RECOVERY_RESTORING))
            {
                if (path_history_pos != 0)
                {
                    path_history_pos--;
                }
                pl_data->feed_rate = PARKING_RATE; //Usually the slower rate
            }
            //make sure we have not reached the end of the available data
            if ((path_history_pos <= (path_history_depth - 1)) && (gap_flags & GAP_FLAG_SHORT_RECOVERY_ACTIVE))
            {
                pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION | planer_restore_condition);
                pl_data->spindle_speed = planer_restore_spindle_speed;
                pl_data->line_number = PARKING_MOTION_LINE_NUMBER;

                printInteger(path_history_pos);
                printPgmString(PSTR(": "));
                system_convert_array_steps_to_mpos(backwards_target, path_history[path_history_pos]);
                report_util_axis_values(backwards_target);
                printString("\r\n");
                bit_true(gap_flags, GAP_FLAG_Moving); //This is needed as the mc_parking_motion can cause this whole funtion to be called again. Need to track if we are still in a move op
                mc_parking_motion(backwards_target, pl_data);
                bit_false(gap_flags, GAP_FLAG_Moving);

                if ((path_history_pos == 0) && (gap_flags & GAP_FLAG_SHORT_RECOVERY_RESTORING))
                {
                    bit_false(gap_flags, GAP_FLAG_SHORT_RECOVERY_ACTIVE);
                    bit_false(gap_flags, GAP_FLAG_SHORT_RECOVERY_RESTORING);
                    system_set_exec_state_flag(EXEC_CYCLE_START); // Set to resume program.
                }
                /*                 if (gap_voltage < min_gap_v_before_clear) //Still shorted
                {
                    path_history_pos++;
                    if (path_history_pos > backwards_buffer_size){path_history_pos = backwards_buffer_size;} //cap out
                    pl_data->feed_rate = PARKING_PULLOUT_RATE; //Usually the faster rate
                } */
            }
            else //we have reached the end of our buffer
            {
                //printString("We're stuck...\r\n");
            }
        }
    }
}
ISR(TIMER5_OVF_vect)
{
    gap_tick();
}
/*
This function acts as a timer so we can control the rate at which we sample the analog pin.
A tick happens on every Timer2 OVF. 1/8 prescaler -> 7.8kHz 
Accurate timing is not critcal however consistant timming is pretty important for the stability of the PI control of feedrate
*/
void gap_tick()
{
    sys_gap_read_state = true;
}