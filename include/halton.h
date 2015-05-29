#include "units.h"
#include <vector>

using namespace std;

px Halton(int index, int base);

ImCoord ImHalton(int index, int base_x, int base_y);

vector<ImCoord> HaltonSeq2D(int start_index,
                            int num_samples,
                            int base_x,
                            int base_y);
