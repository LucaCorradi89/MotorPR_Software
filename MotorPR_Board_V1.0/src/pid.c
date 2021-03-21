#include <pid.h>

/*! \details This function initializes the data in a PID structure.
 *
 */
void pid_init_f(pid_f_t * ptr, float min, float max){
	ptr->min = min;
	ptr->max = max;
}

/*! \details This function updates the value of the manipulated variable (MV)
 * based on the current state of the PID loop.
 */
float pid_update_f(float sp, float pv, pid_f_t * ptr ){
	float e;
	float manp;
	float tmpi;
	e = ptr->e;
	ptr->e = sp - pv;
	tmpi = ptr->i + ptr->e;
	//bound the integral
	manp = ptr->kp * ptr->e + ptr->ki * tmpi + ptr->kd * (e - ptr->e);
	if ( (manp < ptr->max) && (manp > ptr->min) ){
		ptr->i = tmpi;
	} else if ( manp > ptr->max ){
		manp = ptr->max;
	} else if ( manp < ptr->min ){
		manp = ptr->min;
	}
	return manp;
}
