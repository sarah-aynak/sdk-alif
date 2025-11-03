# SPDX-License-Identifier: Apache-2.0

set(ARM_C_FLAGS)

# HACK: Pass correct flags to Clang to make sure object files link with
# -lc, -lm, and -lclang_rt.builtins libraries.
if(CONFIG_CPU_CORTEX_M55 AND (ZEPHYR_TOOLCHAIN_VARIANT STREQUAL "llvm"))
if(CONFIG_FPU)
list(APPEND ARM_C_FLAGS   -mfloat-abi=hard -march=armv8.1m.main+dsp+fp+mve
			  -mthumb -mlittle-endian)
set(GCC_M_FPU none)
else()
list(APPEND ARM_C_FLAGS   -mfloat-abi=soft -march=armv8.1m.main+dsp+fp+mve
			  -mthumb -mlittle-endian)
endif()

# HACK: clang does not generate __ARM_ARCH_8M_MAIN__ but generate
#  __ARM_ARCH_8_1M_MAIN__ instead. The CMSIS_5
# CMSIS/Core/Include/cmsis_gcc.h which is referred to by the clang
# for the low-level functions does not use __ARM_ARCH_8_1M_MAIN__
# that results in undefined references. So, make sure to define
# __ARM_ARCH_8M_MAIN__ when compiling code using clang for M55 until
# zephyr uses CMSIS/Core/Include/cmsis_clang.h of CMSIS_6.
# More info: https://github.com/ARM-software/CMSIS_5/issues/1210
list(APPEND ARM_C_FLAGS   -D__ARM_ARCH_8M_MAIN__=1)
else()
list(APPEND ARM_C_FLAGS   -mcpu=${GCC_M_CPU})
endif()

if(CONFIG_COMPILER_ISA_THUMB2)
  list(APPEND ARM_C_FLAGS   -mthumb)
endif()

list(APPEND ARM_C_FLAGS -mabi=aapcs)

if(CONFIG_FPU)
  list(APPEND ARM_C_FLAGS   -mfpu=${GCC_M_FPU})

  if(CONFIG_DCLS AND NOT CONFIG_FP_HARDABI)
    # If the processor is equipped with VFP and configured in DCLS topology,
    # the FP "hard" ABI must be used in order to facilitate the FP register
    # initialisation and synchronisation.
    set(FORCE_FP_HARDABI TRUE)
  endif()

  if    (CONFIG_FP_HARDABI OR FORCE_FP_HARDABI)
    list(APPEND ARM_C_FLAGS   -mfloat-abi=hard)
  elseif(CONFIG_FP_SOFTABI)
    list(APPEND ARM_C_FLAGS   -mfloat-abi=softfp)
  endif()
endif()

if(CONFIG_FP16)
  # Clang only supports IEEE 754-2008 format for __fp16. It's enabled by
  # default, so no need to do anything when CONFIG_FP16_IEEE is selected.
  if(CONFIG_FP16_ALT)
    message(FATAL_ERROR "Clang doesn't support ARM alternative format for FP16")
  endif()
endif()
list(APPEND TOOLCHAIN_C_FLAGS ${ARM_C_FLAGS})
list(APPEND TOOLCHAIN_LD_FLAGS NO_SPLIT ${ARM_C_FLAGS})
