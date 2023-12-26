# SERVO EASING
Servo easing is an open source used for easing the servo movement.

After I reference to arduino library at [ArminJo/ServoEasing](https://github.com/ArminJo/ServoEasing), the servo has been eased as I expected but when I read to the source, I think it's hard to extend.

This library has been organize with a platform as a controller, you can have more than one controller at the same time. I hope it can support from the Linux base board, to small MCU board so dynamic allocate is only used in Linux base controller and floating point support option will be available soon.

Any contribute or bug report are welcome. Easing algorithms are C implement from source of ArmiJo/ServoEasing.