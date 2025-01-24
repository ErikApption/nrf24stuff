#!/bin/bash

time=`date`
#PWM1_M2=fd8b0010
PWM=/sys/devices/platform/fd8b0010.pwm
echo [${time}] Noob PWM script started, push to 80% for 3 seconds to start
echo 0 > ${PWM}/pwm/pwmchip*/export
echo 10000 > ${PWM}/pwm/pwmchip*/pwm0/period
echo 8000 > ${PWM}/pwm/pwmchip*/pwm0/duty_cycle
echo normal > ${PWM}/pwm/pwmchip*/pwm0/polarity
echo 1 > ${PWM}/pwm/pwmchip*/pwm0/enable
sleep 3

while true
do
	a=`cat /sys/class/thermal/thermal_zone0/temp`
	time=`date`
	pwm=`echo ${a} | awk '{printf("%d",$1*0.15-800)}'`
	if (($a < 40000))
	then
		pwm=1000
	elif (($a < 50000))
	then
		pwm=5000
	else
		pwm=8500
	fi
	# if (( $pwm > 8500 ))
	# then
	# 	pwm=8500
	# elif (( $pwm < 5000 ))
	# then
	# 	pwm=5000
	# fi
	echo ${pwm} > ${PWM}/pwm/pwmchip*/pwm0/duty_cycle
	#echo [${time}] Thermal value: ${a}, write fan PWM cycle ${pwm}
	sleep 5
done