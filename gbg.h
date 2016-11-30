/**************************************************************************
gbg.h
defines to cross the source files
 *************************************************************************/


/**************************************************************************
 *  Defines for hardware.  Use of defines dosn't use ram for storing
 *  ints or other data types
**************************************************************************/
#define REV_RELAY_DRV 2
#define RUN_RELAY_DRV 3
#define REM_LOCAL 5
#define OP_RLY 6
#define POT_DRV 9               /* this is a PWM output  */
                                /* Range 0 to 255  */
/* tbd: check pwm frequncy-- either 490 or 980 Hz */
#define SPEED_POT A0
#define RANGE_SENSOR A1
#define POT_IN A5


/**************************************************************************
Declare variables and functions used in several places
**************************************************************************/
void setupBT(void);             /* Declare for external use */

extern int potValue;
extern int range_val;
extern int cycle_time;

