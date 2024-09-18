#include "Var_str.h"

int can_place_drone(const map* map, const point p) {
  if (p.x < 0 || p.y < 0 || p.x + map->drone_size > map->width ||
      p.y + map->drone_size > map->height) {
    return 0;
  }

  for (int y = p.y; y < p.y + map->drone_size; y++) {
    for (int x = p.x; x < p.x + map->drone_size; x++) {
      int index = x + y * map->width;
      if (index < 0 || index >= map->width * map->height ||
          !map->points[index] || map->points[index]->is_obstacle) {
        return 0;
      }
    }
  }

  return 1;
}

void place_obs(map* map, point a, point b) {
  int dx = 0;
  if (a.x > b.x) {
    dx = -1;
  } else if (a.x < b.x) {
    dx = 1;
  }

  int dy = 0;
  if (a.y > b.y) {
    dy = -1;
  } else if (a.y < b.y) {
    dy = 1;
  }

  point c = a;

  if (c.x < 0 || c.x >= map->width || c.y < 0 || c.y >= map->height) {
    return;
  }

  if (c.x == b.x && c.y == b.y) {
    map->points[c.x + c.y * map->width]->is_obstacle = 1;
    return;
  }

  while (c.x != b.x || c.y != b.y) {
    if (c.x >= 0 && c.x < map->width && c.y >= 0 && c.y < map->height) {
      map->points[c.x + c.y * map->width]->is_obstacle = 1;
    }
    if (c.x == b.x && c.y == b.y) {
      break;
    }
    c.x += dx;
    c.y += dy;
  }

  if (c.x >= 0 && c.x < map->width && c.y >= 0 && c.y < map->height) {
    map->points[c.x + c.y * map->width]->is_obstacle = 1;
  }
}

double heuristic(point a, point b) {
  int dx = abs(a.x - b.x);
  int dy = abs(a.y - b.y);
  return sqrt(dx * dx + dy * dy);
}

void mark_drone_path(map* map, point p) {
  for (int i = 0; i < map->drone_size; i++) {
    for (int j = 0; j < map->drone_size; j++) {
      int index = (p.x + j) + (p.y + i) * map->width;
      map->points[index]->in_path = 1;
    }
  }
}

void sort_neighbors(point_on_map* neighbors[], int count) {
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (neighbors && neighbors[i]->f_cost > neighbors[j]->f_cost) {
        point_on_map* temp = neighbors[i];
        neighbors[i] = neighbors[j];
        neighbors[j] = temp;
      }
    }
  }
}

int Astar(map* map, point start, point goal) {
  stack* open_set = stack_new();
  point_on_map** all_nodes =
      malloc(sizeof *all_nodes * map->width * map->height);

  for (int y = 0; y < map->height; y++) {
    for (int x = 0; x < map->width; x++) {
      point_on_map* p = map->points[x + y * map->width];
      if (!p) {
        fprintf(stderr, "Invalid point at (%d, %d)\n", x, y);
        return 0;
      }
      p->in_path = 0;
      p->last = 0;
      p->g_cost = INFINITY;
      p->h_cost = INFINITY;
      p->f_cost = INFINITY;
      p->next = NULL;
      all_nodes[x + y * map->width] = p;
    }
  }

  point_on_map* start_node = all_nodes[start.x + start.y * map->width];
  start_node->g_cost = 0;
  start_node->h_cost = heuristic(start, goal);
  start_node->f_cost = start_node->h_cost;
  stack_push(&open_set, start_node);

  int directions[8][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
                          {0, 1}, {1, 0},  {0, -1}, {-1, 0}};

  while (open_set->data) {
    point_on_map* current = stack_pop(&open_set);
    if (!current) {
      fprintf(stderr, "Failed to pop element from stack\n");
      break;
    }

    if (current->point.x == goal.x && current->point.y == goal.y) {
      point_on_map* n = current;
      while (n) {
        mark_drone_path(map, n->point);
        n = n->next;
      }
      free(open_set);
      free(all_nodes);
      return 1;
    }

    current->last = 1;

    point_on_map* neighbors[8];
    int neighbor_count = 0;

    for (int i = 0; i < 8; i++) {
      int nx = current->point.x + directions[i][0];
      int ny = current->point.y + directions[i][1];
      point new_point = {nx, ny};

      double direction_cost = (i < 4) ? sqrt(2) : 1.0;

      if (map->drone_size == 1 &&
          (directions[i][0] != 0 && directions[i][1] != 0)) {
        int x1 = nx;
        int y1 = current->point.y;
        int x2 = current->point.x;
        int y2 = ny;

        if (x1 >= 0 && x1 < map->width && y1 >= 0 && y1 < map->height &&
            x2 >= 0 && x2 < map->width && y2 >= 0 && y2 < map->height) {
          if (all_nodes[x1 + y1 * map->width] &&
              all_nodes[x2 + y2 * map->width]) {
            if (all_nodes[x1 + y1 * map->width]->is_obstacle ||
                all_nodes[x2 + y2 * map->width]->is_obstacle) {
              continue;
            }
          }
        }
      }

      if (nx >= 0 && nx < map->width && ny >= 0 && ny < map->height &&
          can_place_drone(map, new_point)) {
        point_on_map* neighbor = all_nodes[nx + ny * map->width];

        if (neighbor->is_obstacle || neighbor->last) {
          continue;
        }

        double tentative_g_cost = current->g_cost + direction_cost;

        if (tentative_g_cost < neighbor->g_cost) {
          neighbor->next = current;
          neighbor->g_cost = tentative_g_cost;
          neighbor->h_cost = heuristic(neighbor->point, goal);
          neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

          neighbors[neighbor_count++] = neighbor;
        }
      }
    }

    sort_neighbors(neighbors, neighbor_count);

    if (neighbor_count) {
      stack_push(&open_set, neighbors[0]);
    }
  }

  free(open_set);
  free(all_nodes);
  return 0;
}

map* Alg() {
  FILE* file = fopen("map.txt", "r");
  if (!file) {
    fprintf(stderr, "Failed to open map.txt\n");
    return NULL;
  }

  int width, height, drone_size, n_obstacles;
  fscanf(file,
         "Map size(W x H): %d x %d\nDrone size: %d\nNumber of obstacles: %d\n",
         &width, &height, &drone_size, &n_obstacles);

  map* map = map_new(width, height, drone_size, (point){.x = 0, .y = 0});
  if (!map) {
    fprintf(stderr, "Failed to create map\n");
    fclose(file);
    return NULL;
  }

  for (int i = 0; i < n_obstacles; i++) {
    point obstacle_a, obstacle_b;
    fscanf(file, "Obstacle coordinates: (%d, %d) (%d, %d)\n", &obstacle_a.x,
           &obstacle_a.y, &obstacle_b.x, &obstacle_b.y);
    place_obs(map, obstacle_a, obstacle_b);
  }

  point start, end;
  fscanf(file, "Start point: (%d, %d)\nEnd point: (%d, %d)\n", &start.x,
         &start.y, &end.x, &end.y);

  map->start_point = map->points[start.x + start.y * map->width];
  if (!map->start_point) {
    fprintf(stderr, "Invalid start point\n");
    fclose(file);
    return NULL;
  }

  int result = Astar(map, start, end);

  fclose(file);

  if (!result) {
    fprintf(stderr, "Path not found\n");
  }

  return map;
}
