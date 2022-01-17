#include "Surface.h"

Surface::Surface(Vector<3> position, Vector<3> normal, double area,
                 double reflectivity, double specularity,
                 double air_specularity)
    : position_(position), normal_(normal), area_(area),
      reflectivity_(reflectivity), specularity_(specularity),
      air_specularity_(air_specularity) {}
