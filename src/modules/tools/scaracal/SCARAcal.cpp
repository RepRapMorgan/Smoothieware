
/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SCARAcal.h"

#include "Kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SerialMessage.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include "StringStream.h"

#include <algorithm>
#include <map>
#include <string>

#define scaracal_checksum CHECKSUM("scaracal")
#define enable_checksum CHECKSUM("enable")
#define slow_feedrate_checksum CHECKSUM("slow_feedrate")
#define z_move_checksum CHECKSUM("z_move")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

void SCARAcal::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( scaracal_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
}

void SCARAcal::on_config_reload(void *argument)
{
    this->slow_rate = THEKERNEL->config->value( scaracal_checksum, slow_feedrate_checksum )->by_default(5)->as_number(); // feedrate in mm/sec
    this->z_move = THEKERNEL->config->value( scaracal_checksum, z_move_checksum )->by_default(0)->as_number(); // Optional movement of Z relative to the home position.
                                                                                                  // positive values increase distance between nozzle and bed.
                                                                                                  // negative will decrease.  Useful when level code active to prevent collision
    // TODO: Init procal with values from config
    this->calpos = 99;
}


// issue home command
void SCARAcal::home()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

bool SCARAcal::get_trim(float& x, float& y, float& z)
{
    void *returned_data;
    bool ok = PublicData::get_value( endstops_checksum, trim_checksum, &returned_data );

    if (ok) {
        float *trim = static_cast<float *>(returned_data);
        x= trim[0];
        y= trim[1];
        z= trim[2];
        return true;
    }
    return false;
}

bool SCARAcal::set_trim(float x, float y, float z, StreamOutput *stream)
{
    float t[3]{x, y, z};
    bool ok= PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
        stream->printf("Set trim to X:%f Y:%f Z:%f\n", x, y, z);
    } else {
        stream->printf("Unable to set trim, is endstops enabled?\n");
    }

    return ok;
}

bool SCARAcal::get_home_offset(float& x, float& y, float& z)
{
    float returned_data[3];
    bool ok = PublicData::get_value( endstops_checksum, home_offset_checksum, returned_data );

    if (ok) {
        x= returned_data[0];
        y= returned_data[1];
        z= returned_data[2];
        return true;
    }
    return false;
}

bool SCARAcal::set_home_offset(float x, float y, float z, StreamOutput *stream)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "M206 X%1.3f Y%1.3f Z%1.3f", x, y, z); // Send saved Z homing offset

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

    stream->printf("Set home_offset to X:%f Y:%f Z:%f\n", x, y, z);

    return true;//ok;
}

bool SCARAcal::translate_trim(StreamOutput *stream)
{
    float S_trim[3],
        home_off[3];
    ActuatorCoordinates actuator;

    this->get_home_offset(home_off[0], home_off[1], home_off[2]);               // get home offsets
    this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	                          // get current trim

    THEROBOT->arm_solution->cartesian_to_actuator( home_off, actuator ); // convert current home offset to actuator angles

    // Subtract trim values from actuators to determine the real home offset actuator position for X and Y.

    actuator[0] -= S_trim[0];
    actuator[1] -= S_trim[1];

    // Clear X and Y trims internally
    S_trim[0] = 0.0F;
    S_trim[1] = 0.0F;

    // convert back to get the real cartesian home offsets

    THEROBOT->arm_solution->actuator_to_cartesian( actuator, home_off );

    this->set_home_offset(home_off[0], home_off[1], home_off[2],stream);               // get home offsets
                 // Set the correct home offsets;

    this->set_trim(S_trim[0], S_trim[1], S_trim[2], stream);    // Now Clear relevant trims

    return true;
}

void SCARAcal::SCARA_ang_move(float theta, float psi, float z, float feedrate)
{
    float cartesian[3];
    ActuatorCoordinates actuator;

    // Assign the actuator angles from input
    actuator[0] = theta;
    actuator[1] = psi;
    actuator[2] = z;

    // Calculate the physical position relating to the arm angles
    THEROBOT->arm_solution->actuator_to_cartesian( actuator, cartesian );

    this->SCARA_abs_move(cartesian[0], cartesian[1], cartesian[2], feedrate);
}

void SCARAcal::SCARA_abs_move(float x, float y, float z, float feedrate)
{
    char cmd[64];
    
    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", x, y, z, feedrate * 60); // use specified feedrate (mm/sec)

    this->SCARA_send_move(cmd);
}

void SCARAcal::SCARA_send_move(char* cmd)
{
    // DEBUG 
    THEKERNEL->streams->printf("%s\n",cmd);

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEROBOT->on_gcode_received(&gc); // send to robot directly
}

//A GCode has been received
//See if the current Gcode line has some orders for us
void SCARAcal::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        switch( gcode->m ) {

            case 114: {    // Extra stuff for Morgan calibration
                char buf[32];
                float cartesian[3];
                ActuatorCoordinates actuators;

                THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                THEROBOT->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate to get actuator position

                int n = snprintf(buf, sizeof(buf), "  A: Th:%1.3f Ps:%1.3f",
                                 actuators[0],
                                 actuators[1]);    // display actuator angles Theta and Psi.
                gcode->txt_after_ok.append(buf, n);
            }
            break;

            case 360: {
                float target[2] = {0.0F, 150.0F},
                      cartesian[3],
                      S_trim[3];

                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	// get current trim to conserve other calbration values

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    ActuatorCoordinates actuators;
                    float S_delta[2],
                          S_trim[3];

                    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                    THEROBOT->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate to get actuator position

                    S_delta[0] = actuators[0] - target[0];

                    set_trim(S_delta[0], S_trim[1], S_trim[2], gcode->stream);
                } else {
                    set_trim(0, S_trim[1], S_trim[2], gcode->stream);               // reset trim for calibration move
                    this->home();                                                   // home
                    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                    SCARA_ang_move(target[0], target[1], cartesian[2] + this->z_move, slow_rate * 3.0F); // move to target
                }
            }
            break;

            case 361: {
                float target[2],
                      cartesian[3];

                if(gcode->subcode == 1){ // 180 degree condition
                    THEROBOT->software_limits = false;

                    target[0] = 180.0F;
                    target[1] = 280.0F;
                } else {
                    target[0] = 90.0F;
                    target[1] = 130.0F;
                }
                gcode->stream->printf("Target: T %f P %f\n", target[0], target[1]);

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    ActuatorCoordinates actuators;

                    THEROBOT->get_axis_position(cartesian);                                // get actual position from robot
                    THEROBOT->arm_solution->cartesian_to_actuator( cartesian, actuators ); // translate to get actuator position

                    STEPPER[0]->change_steps_per_mm(actuators[0] / target[0] * STEPPER[0]->get_steps_per_mm()); // Find angle difference
                    STEPPER[1]->change_steps_per_mm(STEPPER[0]->get_steps_per_mm());  // and change steps_per_mm to ensure correct steps per *angle*

                    SCARA_ang_move(120.0F, 220.0F, cartesian[2], slow_rate * 3.0F); // move to safe position after setting steps
                } else {
                    this->home();                                                   // home - This time leave trims as adjusted.
                    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                    SCARA_ang_move(target[0], target[1], cartesian[2] + this->z_move, slow_rate * 3.0F); // move to target
                }
            }
            break;

            case 362: {
                float target[2] = {0.0F, 90.0F},
                      cartesian[3],
                      S_trim[3];

                THEROBOT->software_limits = false;

                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);    // get current trim to conserve other calbration values

                set_trim(S_trim[0], 0, S_trim[2], gcode->stream);               // reset trim for calibration move
                this->home();                                                   // home
                THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                SCARA_ang_move(target[0], target[1], cartesian[2] + this->z_move, slow_rate * 3.0F); // move to target

            }
            break;

            case 363: {
                float target[2] = {180.0F, 270.0F},
                      cartesian[3];

                THEROBOT->software_limits = false;
                gcode->stream->printf("Software limis = %i\n", THEROBOT->software_limits);

                gcode->stream->printf("Target: T %f P %f\n", target[0], target[1]);

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    ActuatorCoordinates actuators;

                    THEROBOT->get_axis_position(cartesian);                                // get actual position from robot
                    THEROBOT->arm_solution->cartesian_to_actuator( cartesian, actuators ); // translate to get actuator position

                    STEPPER[1]->change_steps_per_mm(actuators[1] / target[1] * STEPPER[1]->get_steps_per_mm()); // Find angle difference
                    //STEPPER[1]->change_steps_per_mm(STEPPER[0]->get_steps_per_mm());  // and change steps_per_mm to ensure correct steps per *angle*

                    SCARA_ang_move(120.0F, 220.0F, cartesian[2], slow_rate * 3.0F); // move to safe position after setting steps
                } else {
                    this->home();                                                   // home - This time leave trims as adjusted.
                    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                    SCARA_ang_move(target[0], target[1], cartesian[2] + this->z_move, slow_rate * 3.0F); // move to target
                }

            }
            break;

            case 364: {
                float target[2],
                      cartesian[3],
                      S_trim[3];
                
                if(gcode->subcode == 1){ // 180 degree condition
                    THEROBOT->software_limits = false;

                    target[0] = 135.0F;
                    target[1] = 135.0F;
                } else {
                    target[0] = 45.0F;
                    target[1] = 135.0F;
                }
                
                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	// get current trim to conserve other calbration values

                if(gcode->has_letter('P')) {
                    float pval = gcode->get_value('P');
                        // Program the current position as target
                        ActuatorCoordinates actuators;
                        float S_delta[2];

                        THEROBOT->get_axis_position(cartesian);                                     // get actual position from robot
                        THEROBOT->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate it to get actual actuator angles

                    if (pval == 0.0) {
                    
                        S_delta[1] = ( actuators[1] - actuators[0]) - ( target[1] - target[0] );            // Find difference in angle - not actuator difference, and
                        
                    } else {
                        // Use the value of the entry to set the actual angle
                        S_delta[1] = ( actuators[1] - actuators[0]) - pval ;

                    }

                    set_trim(S_trim[0], S_delta[1], S_trim[2], gcode->stream);                                  // set trim to reflect the difference

                } else {
                    set_trim(S_trim[0], 0, S_trim[2], gcode->stream);                                           // reset trim for calibration move
                    this->home();                                                                       // home
                    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
                    this->SCARA_ang_move(target[0], target[1], 50, slow_rate * 3.0F);                     // move to target
                }
            }
            break;

            case 365:
            // Run arm calbration probe routine
                

                if(gcode->subcode == 1){   // Continue operation after moving arm into position
                    if (calpos<11){
                        char cmd[64];
                        float cartesian[3], pos[3];
                        int axis, probe_dir;
                        StringStream string_stream;
                        
                        //switch(axis) {
                        //    case X_AXIS: cmd.append(probe_dir ? "X50" : "X-50"); break;
                        //    case Y_AXIS: cmd.append(probe_dir ? "Y50" : "Y-50"); break;
                        //    case Z_AXIS: cmd.append(probe_dir ? "Z50" : "Z-50"); break;
                        //}

                        // User moved the touch probe to better position manually if needed
                        // Probe finds X center, then Y center and repeats.
                        promeasure[calpos][X_AXIS] = NAN;
                        promeasure[calpos][Y_AXIS] = NAN;

                        for (axis = 0; axis < 2; axis++){
                            promeasure[calpos][axis] = 0.0;
                            for (probe_dir = 0; probe_dir < 2; probe_dir++){
                                THEROBOT->get_axis_position(cartesian);                                     // get actual position from robot
                        
                                //string cmd("G38.3 X8");                       
                                snprintf(cmd, sizeof(cmd), "G38.2 %c%s", 'X' + axis, probe_dir ? "-8" : "8"); // 
                                //snprintf(cmd, sizeof(cmd), "G38.2 X8"); // 
                                //cmd.append("X8");
                            
                                //this->SCARA_send_move(cmd);
                            
                                // first wait for all moves to finish
                                //THEKERNEL->conveyor->wait_for_idle();

                                struct SerialMessage message;
                                message.message = cmd;
                                message.stream = &string_stream;
                                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                                if(THEKERNEL->is_halted()) return;
                            
                                THEROBOT->get_axis_position(pos, 3);   // get actual value again to deduce probe position
                                promeasure[calpos][axis] += pos[axis]; // add the values
                            
                                
                                
                                 
                            
                                this->SCARA_abs_move(cartesian[0],cartesian[1],NAN,500); // move back for second move
                            }
                            

                            THEROBOT->get_axis_position(pos, 3);
                            promeasure[calpos][axis] /= 2;
                            this->SCARA_abs_move(axis ? NAN : promeasure[calpos][X_AXIS], axis ? promeasure[calpos][Y_AXIS] : NAN,NAN,500); // move to new center
                        }


                        calpos++;
                        this->SCARA_abs_move(NAN,NAN,20,500); // Lift the head first
                        if (calpos < 11){
                            this->SCARA_abs_move(procal[calpos][0],procal[calpos][1],20,500); // Move to new position
                        };



                    } else if (calpos==11){

                        int i;

                        gcode->stream->printf("\nXsol,Ysol,Xist,Yist\n");

                        for (i=0; i<11; i++){
                            gcode->stream->printf("%i,%i,%f,%f\n", procal[i][X_AXIS],procal[i][Y_AXIS],promeasure[i][X_AXIS], promeasure[i][Y_AXIS]);


                            
                        }
                        calpos = 99;
                    }



                } else {
                    if(gcode->has_letter('P')) {
                        float pval = gcode->get_value('P');

                        if (pval == 2014){
                            // Initialize calibrator. Ensure that the calbrator tip Z has been set to the calibrator plate
                            // This will move the arm to the first calibration position, 10mm above the calibrator plate.

                            // move the tip close to the center of the calibration point if it is not close, and run:
                            // M365.1

                            this->calpos = 0; // Start position

                            this->home(); // Home the machine
                            this->SCARA_abs_move(procal[calpos][0],procal[calpos][1],20,1000);  //move to first position
                            
                        }

                    }


                }
            break;

            case 366:                                       // Translate trims to the actual endstop offsets for SCARA
                THEROBOT->software_limits = true;
                this->translate_trim(gcode->stream);
                break;

        }
    }
}

