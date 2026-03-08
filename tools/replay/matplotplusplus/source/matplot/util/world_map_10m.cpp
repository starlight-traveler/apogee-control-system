//
// Local fallback for repos that vendor Matplot++ without the generated 10m map blob.
//

#include <matplot/util/geodata.h>

namespace matplot {

std::pair<std::vector<double>, std::vector<double>> &world_map_10m() {
    return world_map_50m();
}

}
