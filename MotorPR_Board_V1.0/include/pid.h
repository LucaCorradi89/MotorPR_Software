#ifndef PID_H_
#define PID_H_

typedef struct{
	float max /*! Max manipulated value */;
	float min /*! Miniumum manipulated value */;
	float e /*! Error value */;
	float i /*! Integrator value */;
	float kp /*! Proportional constant */;
	float ki /*! Integrator constant */;
	float kd /*! Differential constant */;
} pid_f_t;

//Prototipi
void pid_init_f(pid_f_t * ptr, float min, float max);
float pid_update_f(float sp, float pv, pid_f_t * ptr );




#endif /* PID_H_ */
