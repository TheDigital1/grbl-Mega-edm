
// initialization routine.
void InitADC();
uint16_t ReadADC(uint8_t ADCchannel);
void gap_init();
void gap_tick();
void gap_update_feedrate();
void gap_add_pos_to_history();
//This routine backs up in the path attempting to clear a short condition
void gap_clear_short(float adc_val);
int32_t path_history[backwards_buffer_size][N_AXIS];
uint8_t path_history_depth;
uint8_t path_history_pos;
uint8_t planer_restore_condition; 
float planer_restore_spindle_speed;
float backwards_target[N_AXIS];
//int32_t old_position[N_AXIS];
uint8_t gap_flags;
float ave_gap_voltage;
// Define gap control flags. 
#define GAP_FLAG_VCONTROL_ENABLED      bit(0)
#define GAP_FLAG_SHORT_RECOVERY_ACTIVE      bit(1)
#define GAP_FLAG_SHORT_RECOVERY_RESTORING      bit(2)
#define GAP_FLAG_Moving      bit(3)
// Backwards Step segment ring buffer indices
//uint8_t backwards_target_buffer_count;
//uint8_t backwards_target_buffer_head;
//uint8_t backwards_target_buffer_next_head;
//uint8_t backwards_target_buffer_start;
//uint8_t backwards_target_buffer_intr;
