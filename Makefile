# --- hardware build (disting NT plug-in) -------------------------------------------------------
# Point API at the include/ folder of a checkout of https://github.com/expertsleepersltd/distingNT_API
# If the API repo's own example Makefile uses different flags, prefer those.
API      ?= ../distingNT_API/include
CXX      ?= arm-none-eabi-c++
CXXFLAGS := -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb \
            -fno-rtti -fno-exceptions -Os -fPIC -Wall -I$(API) -I.

plugins/comb_voice.o: comb_voice.cpp comb_core.h
	mkdir -p plugins
	$(CXX) $(CXXFLAGS) -c -o build_comb_voice.o comb_voice.cpp
	$(CXX) $(CXXFLAGS) -Wl,--relocatable -nostdlib -o $@ build_comb_voice.o
	rm -f build_comb_voice.o

# --- host tests (use a stand-in API header; they do NOT validate against the real one) ------------
test: render test_wrapper
	python3 analyze.py
	./test_wrapper

render: render.cpp comb_core.h
	g++ -O2 -Wall -Wextra -o $@ $<

test_wrapper: comb_voice.cpp test_wrapper.cpp comb_core.h mock/distingnt/api.h
	g++ -std=c++11 -O2 -Wno-unused-parameter -Wno-missing-field-initializers -Imock -o $@ comb_voice.cpp test_wrapper.cpp

.PHONY: test
