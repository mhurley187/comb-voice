# Disting NT plug-ins: Comb Voice, Phase Six, Spectral Delay and Allpass Smear. Same flags as the official distingNT_API examples.
API      ?= ../distingNT_API/include
ARM_CXX  ?= arm-none-eabi-c++
CXXFLAGS := -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -fno-rtti \
            -fno-exceptions -fno-tree-loop-distribute-patterns -Os -fPIC -Wall -I$(API) -I.

all: plugins/comb_voice.o plugins/phase_six.o plugins/spectral_delay.o plugins/allpass_smear.o

plugins/comb_voice.o: comb_voice.cpp comb_core.h nt_math.h
	mkdir -p plugins
	$(ARM_CXX) $(CXXFLAGS) -c -o $@ comb_voice.cpp

plugins/phase_six.o: phase_six.cpp lfo_core.h nt_math.h
	mkdir -p plugins
	$(ARM_CXX) $(CXXFLAGS) -c -o $@ phase_six.cpp

plugins/spectral_delay.o: spectral_delay.cpp spectral_core.h nt_math.h
	mkdir -p plugins
	$(ARM_CXX) $(CXXFLAGS) -c -o $@ spectral_delay.cpp

plugins/allpass_smear.o: allpass_smear.cpp allpass_core.h nt_math.h
	mkdir -p plugins
	$(ARM_CXX) $(CXXFLAGS) -c -o $@ allpass_smear.cpp

.PHONY: all
