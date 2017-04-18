/**************************************************************************
gbg.h
defines to cross the source files
 *************************************************************************/


/**************************************************************************
 *  Defines for hardware.  Use of defines dosn't use ram for storing
 *  ints or other data types
 *
 *  tbd, some of the inputs will be changed over to analog to free
 *  digital outputs
**************************************************************************/
#define REV_RELAY_DRV 2         // output
#define RUN_RELAY_DRV 3         // output

#define REM_LOCAL 5             // input
#define SPARE 6                // input
#define REVERSE 10


#define POT_DRV 9               /* this is a PWM output  */
                                /* Range 0 to 255
                                 * this is not used this rev
                                 * but may be later*/

                                /* tbd: check pwm frequncy-- either 490 or 980 Hz */
#define SPEED_POT A0
#define RANGE_SENSOR A1
#define OP_1 A5                 // using analog in to read operator
                                // switch.
                                // 1 = open, not moving
                                // 0 = closed moving
                                // 1 is 5v or above 1024/2
                                // 0 is 0V or below 1024/2


/**************************************************************************
 * Testmode flag is to control some of the compile options for testing
 * hardware without a phone in the loop
 * tbd RUNMANUAL will have to include the rem local switch
**************************************************************************/

#define TESTPHONE 1             // use to test or run the phone
#define TESTIO 2                // to test IO
#define RUNMANUAL 4             // to run manual


/**************************************************************************
 * The phone app (Android) has 8 buttons at the moment.
 * TBD look at the code sometime.
 * Buttons with commit are named, else they are numbered.
**************************************************************************/
/**************************************************************************
 * Button definitons
 * Each button may be pressed in combo with others, if they make sense
 * tbd lock out some buttons based on others, probably a mask
 * Definitions are 
**************************************************************************/

#define BT_FWD 0x01 << 1
#define BT_SLOWER 0x01 << 2
#define BT_STOP 0x01 << 3
#define BT_START 0x01 << 4
#define BT_FASTER 0x01 << 5
#define BT_REV 0x01 << 6
#define BT_LIGHT 0x01 << 7
#define BT_HORN 0x01 << 8

/**************************************************************************
 * Actions will relate to the bottons pressed.  These may be used to
 * form a status for the car at a particular time and may determine
 * actions based on other inputs.
 * The bit pattern is shown after each 
**************************************************************************/

#define ACT_FWD 0x01 << 1       // 0x02
#define ACT_SLOWER 0x01 << 2    // 0x04
#define ACT_STOP 0x01 << 3      // 0x08 
#define ACT_START 0x01 << 4     // 0x10 
#define ACT_FASTER 0x01 << 5    // 0x20 
#define ACT_REV 0x01 << 6       // 0x40 
#define ACT_LIGHT 0x01 << 7     // 0x80 
#define ACT_HORN 0x01 << 8      // 0x100


/**************************************************************************
Declare variables and functions used in several places
**************************************************************************/
void setupBT(void);             /* Declare for external use */

extern int potValue;
extern int range_val;
extern int cycle_time;

