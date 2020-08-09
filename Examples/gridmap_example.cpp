#include <iostream>
#include "grid_map_core/GridMap.hpp"

using namespace std;
using namespace grid_map;

int main() {

    GridMap map( { "types" });
    map.setGeometry(Length(3.0, 3.0), 0.01, Position(0.0, 0.0));

    cout << 111111111 << endl;
    map.~GridMap();
    return 0;
}