# IR_receiver-NEC

This library allows you to comfortably decode NEC protocol used by remote IR controls.

## Hardware support

ATtiny85: 8-bit Timer/Counter0, INT0

ATtiny88: 8-bit Timer/Counter0, INT0

CPU clock is 8 MHz.

## How to use

- In file `IR_NEC.c` you can modify pulse durations according to your measurements (e.g. by oscilloscope) and pulse thresholds (min. and max. durations for each pulse) using pulse tolerance multiplier (use in interval from 1 to 2; the greater the multiplier, the greater the tolerance).

- Connect output pin of IR receiver to input pin PB2->ATtiny85 / PD2->ATtiny88 (pin 7/4).

-  Execute `IR_init(uint32_t bufferTimeout)` function to initialize IR receiving and set buffer timeout. When timeout occurs, IR data won't be accessible. Before timeout the last received IR data are stored. Set buffer timeout according to delay which is caused by main program. For zero delay write `IR_init(0)`. Actually buffer timeout has some min. value which is necessary, because of time between individual IR codes.

- Repetition for certain commands can be disabled using `IR_disable_repetition(uint8_t command)`. For each command one `IR_disable_repetition(uint8_t command)` execution is needed. So while holding a button, your instructions related to the command are executed just once (`IR_data_ready()` returns false). Disabled repetition can be enabled using `IR_enable_repetition(uint8_t command)` again.

- Then just check `IR_data_ready()` function in loop. If `IR_data_ready()` returns true, IR signal has been checked succesfully and then IR data (address and command) are available. Access to IR data is through `IR_data` structure named IR (`IR.address` for address, `IR.command` for command).

- If `IR_available()` returns true, IR signal has been checked succesfully and then IR data (address and command) are available.

- Access to IR data is through `IR_data` structure named IR (`IR.address` for address, `IR.command` for command).

- Make sure to execute `sei()` function to enable global interrupts.
