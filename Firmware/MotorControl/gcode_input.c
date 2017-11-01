#include "grbl.h"
#include "gcode_input.h"

void consume_output() {
  // Gets the current block. Returns NULL if buffer empty
  plan_block_t *pb = plan_get_current_block();

  if (pb) {
    // We don't do timing here.   We use the actual position to regulate time.
    float real_steps[2];
    real_steps[0] = pb->steps[0] * (pb->direction_bits&1)?1:-1;
    real_steps[1] = pb->steps[1] * (pb->direction_bits&2)?1:-1;
    
    // Position setpoint is the position on the line nearest the current location.
    
    // TODO: For now, we (wrongly) just use the end location as the setpoint.
    motors[0].pos_setpoint += real_steps[0];
    motors[1].pos_setpoint += real_steps[1];


    // Velocity setpoint is the motion profile in https://github.com/gnea/grbl/blob/master/grbl/stepper.c#L172

    // TODO:  For now, we just use the exit velocity for the whole line.
    float vel_sqr = plan_get_exec_block_exit_speed_sqr();
    float dist_sqr = pb->steps[0] * pb->steps[0] + pb->steps[1] * pb->steps[1];
    motors[0].vel_setpoint = real_steps[0] * sqrt(vel_sqr/dist_sqr);
    motors[1].vel_setpoint = real_steps[1] * sqrt(vel_sqr/dist_sqr);

    // Wait for motors to get to/past destination
    // TODO:  Make low_level.c have 'move along a line' functionality, with RTOS events when 
    // a line segment is complete.
    while(true) {
        float dot_product = 
          (motors[0].pos_setpoint - motors[0].encoder.pll_pos) * (real_steps[0]) + 
          (motors[1].pos_setpoint - motors[1].encoder.pll_pos) * (real_steps[1]);
        if (dot_product<=0) break;

        // THIS IS A WHILE TRUE BUSY LOOP...  EXPECT TROUBLE
    }
    plan_discard_current_block();
  }
}



void parse_gcode_line(unsigned char* line, SerialPrintf_t response_interface) {
    static int inited = 0;

    if (!inited) {
        grbl_init();
        inited=1;
    }

    if (!line[0]) return;

    int status = gc_execute_line((char*)line);

    if (status == STATUS_OK) {
        printf("ok\r\n");
    } else {
        printf("error:%d\r\n", status);       
    }
    
}