HasberryPi
==========

Haskell bindings for the wiringPi C library

.hsc to hs
-----------
The hsc2hs command must be run on the .hsc file to generate the actual bindings. The hsc2hs command will generate the HasberryPi.hs file. If you have the Haskell Platform installed, then you will have the hsc2hs command available at the prompt. The HasberryPi.hs file is available in the repository for the purpose of convenience. But it is recommended that you run the hsc2hs tool on the .hsc file to generate the .hs file.

Thread functions are not ported
-------------------------------
There are a few functions in the wiringPi 'C' library that have not been ported. They are
 * extern int piThreadCreate (void *(*fn)(void *));
 * extern void piLock        (int key);
 * extern void piUnlock      (int key);

Serial, I2C, SPI, LCD, SERVO, GERTBOARD
-----------------------------------------------
The other functions in the wiringPi library for which HasberryPi bindings have not been written yet .. are,

gertboard.h
lcd.h
softPwm.h
softServo.h
softTone.h
wiringPiI2C.h
wiringPiSPI.h
wiringSerial.h
wiringShift.h

Remember that the bindings are experimental and may not work as expected.
