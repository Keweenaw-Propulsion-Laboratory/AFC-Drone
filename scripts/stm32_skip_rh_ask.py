"""Drop RadioHead's ASK driver from the STM32 build.

RH_ASK.cpp instantiates HardwareTimer(TIM1) on any stm32duino board, and the
STM32L152 has no TIM1, so the file fails to compile. This project only uses the
RF69 driver, and RadioHead offers no build flag to opt out of RH_ASK, so the
source is filtered out of the library build instead.
"""

Import("env")


def skip_rh_ask(node):
    if node.name == "RH_ASK.cpp":
        return None
    return node


env.AddBuildMiddleware(skip_rh_ask, "*/RadioHead/*")
