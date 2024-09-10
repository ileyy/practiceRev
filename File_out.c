#include "Var_str.h"

void map_print(const map* map) {
  for (int y = 0; y < map->height; y++) {
    for (int x = 0; x < map->width; x++) {
      point_on_map* p = map->points[x + y * map->width];
      if (p->in_path) {
        printf("# ");
      } else if (p->is_obstacle) {
        printf("* ");
      } else {
        printf(". ");
      }
    }
    printf("\n");
  }
}

void output(map* map) { map_print(map); }