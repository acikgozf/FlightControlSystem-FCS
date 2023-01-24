/*
 * bldc_motor_driving.h
 *
 *  Created on: Jan 21, 2023
 *      Author: FURKAN
 */

#ifndef FILTERS_AND_CONTROLLERS_INC_BLDC_MOTOR_DRIVING_H_
#define FILTERS_AND_CONTROLLERS_INC_BLDC_MOTOR_DRIVING_H_


/*
 * Esc protocol consist of Oneshot, Multishot, Dshot, Proshot(signal types)..
 * Esc does not drive to by itself but drives the motor using a signal from the outside.
 * Commonly used firmware are BLHeli_S, SimonK and KISS in ESC
 *
 * ESC nin fimware'ne göre ESC sinyalini ayarla.
 *
 * ESC Protocol types:
 * 						|--				------Duty Cycle-------		------Comparsion------
 * 						| Standard PWM	1000us~2000us (1Khz~500Hz)			?
 * 				PWM->	| Oneshot125	125us~250us   (8Khz~4KHz)		x8 faster than Standard PWM
 * 						| Oneshot42		42us~84us     (24Khz~12KHz)		x3 faster than OneShot125
 * 						| Multishot		5us~25us      (200Khz~40KHz)	x3.4 faster than OneShot142
 * 						|--
 *
 * 				   		|--s
 * 			Digital->   | Dshot
 * 				   		| Proshot
 *						|--
 *	ESC kalibrasyonu yöntemi :Motorun max ve min hızları bir kere ayarlarnmalı esc calibrasyonu tarafından.
 *								Bu ayar ESC nin tanıyabilceği max ve min pulse width anlamına gelir.
 *								Kalibrasyondan sonra tanımlanan pulse width aralığında motor kontrol edilir.
 *								Aralığın dışına cikilirsa motor calismayabilir.
 *
 *	OneShot 125 Prootokü kullanılacak
 *		PWM = 2KHz
 *		Pulse width = 125us-250us
 *		Step between min and max is 10.000
 *
 *	2 kanal PWM uretilecek.
 *
 *	PID kontrolü 1kHz'de yapılacaktır.PWM frekansı bundan daha hızlı olmalıdır..
 *
 *
 *	Darbe genişliğini 250us'a ayarlamak için CCR'yi 10500'ün iki katı olan 21000 olarak ayarlayabilirsiniz
 *	CCR = 10500 ise 125us
 *	CCR = 21000 ise 250us
 *
 *	!!!!
 *	CCR, 0 ile ARR arasında bir değere sahip olmalıdır ve maksimum değer ARR değeri olacaktır.
 * 	Bundan daha yüksek olsa bile, darbe genişliği% 100'dür, yani Yüksek olmaya devam eder
 *
 */

#include "main.h"

void esc_initialize(void);
void esc_control(uint16_t rc_level);



#endif /* FILTERS_AND_CONTROLLERS_INC_BLDC_MOTOR_DRIVING_H_ */
