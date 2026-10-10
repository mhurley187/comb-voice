// host test renderer: render <type> <mut> <volts> <fb> <damp> <freeze> <pluck> <secs> <outfile> [spread chaos]
#include "comb_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
int main(int argc, char** argv)
{
	if (argc < 10) return 1;
	comb::Params P;
	P.typeA = atoi(argv[1]); P.mutA = atoi(argv[2]); P.morph = 0.f;
	float volts = atof(argv[3]);
	P.feedback = atof(argv[4]); P.damping = atof(argv[5]);
	P.freeze = atoi(argv[6]) != 0; P.pluckMode = atoi(argv[7]) != 0;
	float secs = atof(argv[8]);
	P.spreadCents = argc > 10 ? atof(argv[10]) : 0.f;
	P.chaos = argc > 11 ? atof(argv[11]) : 0.f;
	P.excLevel = 0.7f; P.mutAmount = 0.5f; P.excDecayMs = 30.f;
	int n = (int)(secs * 48000);
	std::vector<float> dram(comb::kDramFloats), L(n), R(n), cv(n, volts), gate(n, 0.f);
	for (int i = 0; i < 100 && i < n; ++i) gate[i] = 5.f;     // one trigger at start
	comb::Voice v; v.init(48000.f, dram.data());
	if (P.freeze) { // run unfrozen for 0.3 s, then freeze
		comb::Params Q = P; Q.freeze = false;
		int m = 14400; v.process(Q, cv.data(), gate.data(), nullptr, L.data(), R.data(), m);
		v.process(P, cv.data(), nullptr, nullptr, L.data() + m, R.data() + m, n - m);
	} else v.process(P, cv.data(), gate.data(), nullptr, L.data(), R.data(), n);
	FILE* f = fopen(argv[9], "wb"); fwrite(L.data(), 4, n, f); fwrite(R.data(), 4, n, f); fclose(f);
	return 0;
}
