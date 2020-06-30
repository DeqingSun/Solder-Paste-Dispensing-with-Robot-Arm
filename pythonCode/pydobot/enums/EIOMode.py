from enum import IntEnum


class EIOMode(IntEnum):

    IO_FUNC_DUMMY = 0x00
    IO_FUNC_DO = 0x01
    IO_FUNC_PWM = 0x02
    IO_FUNC_DI = 0x03
    IO_FUNC_ADC = 0x04
    IO_FUNC_DIPU = 0x05
    IO_FUNC_DIPD = 0x06

