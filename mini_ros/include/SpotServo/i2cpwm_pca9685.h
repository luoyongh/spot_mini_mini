#ifndef I2CPWM_PCA9685_INCLUDE_GUARD_H
#define I2CPWM_PCA9685_INCLUDE_GUARD_H

int pca9685_init (const char* filename);
void set_active_board (int board);
void set_pwm_frequency (int freq);
void set_pwm_interval (int servo, int start, int end);

#endif