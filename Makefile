# --- hardware build (disting NT plug-in) -------------------------------------------------------
# Same compiler flags as Expert Sleepers' own distingNT_API examples/Makefile: compile to a plain
# object file with -c (no extra relink step).
# API = include/ folder of a checkout of https://github.com/expertsleepersltd/distingNT_API
API      ?= ../distingNT_API/include
ARM_CXX  ?= arm-none-eabi-c++
CXXFLAGS := -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -fno-rtti \
            -fno-exceptions -Os -fPIC -Wall -I$(API) -I.

plugins/comb_voice.o: comb_voice.cpp comb_core.h nt_math.h
	mkdir -p plugins
	$(ARM_CXX) $(CXXFLAGS) -c -o $@ comb_voice.cpp

# --- host tests (use a stand-in API header; they do NOT validate against the real one) ------------
test: render test_wrapper test_math
	./test_math
	python3 analyze.py
	./test_wrapper

render: render.cpp comb_core.h nt_math.h
	g++ -O2 -Wall -Wextra -o $@ $<

test_math: test_math.cpp nt_math.h
	g++ -O2 -Wall -o $@ $<

test_wrapper: comb_voice.cpp test_wrapper.cpp comb_core.h nt_math.h mock/distingnt/api.h
	g++ -std=c++11 -O2 -Wno-unused-parameter -Wno-missing-field-initializers -Imock -o $@ comb_voice.cpp test_wrapper.cpp

.PHONY: test
