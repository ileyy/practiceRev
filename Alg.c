#include "Var_str.h"

int can_place_drone(const map* map, const point p) {
  if (p.x < 0 || p.y < 0 || p.x + map->drone_size > map->width ||
      p.y + map->drone_size > map->height) {
    return 0;
  }
  for (int i = 0; i < map->drone_size; i++) {
    for (int j = 0; j < map->drone_size; j++) {
      int index = p.x + j + (p.y + i) * map->width;
      if (index < 0 || index >= map->width * map->height ||
          !map->points[index]) {
        return 0;
      }
      if (map->points[index]->is_obstacle) {
        return 0;
      }
      for (int y = p.y; y < p.y + map->drone_size; y++) {
        for (int x = p.x; x < p.x + map->drone_size; x++) {
          if ((x > 0 && y > 0 &&
               map->points[(y - 1) * map->width + x]->is_obstacle &&
               map->points[y * map->width + (x - 1)]
                   ->is_obstacle) ||  // up left
              (x < map->width - 1 && y > 0 &&
               map->points[(y - 1) * map->width + (x + 1)]->is_obstacle &&
               map->points[y * map->width + (x + 1)]
                   ->is_obstacle) ||  // up right
              (x > 0 && y < map->height - 1 &&
               map->points[(y + 1) * map->width + (x - 1)]->is_obstacle &&
               map->points[(y + 1) * map->width + x]
                   ->is_obstacle) ||  // low left
              (x < map->width - 1 && y < map->height - 1 &&
               map->points[(y + 1) * map->width + (x + 1)]->is_obstacle &&
               map->points[(y + 1) * map->width + x]
                   ->is_obstacle)) {  // low right
            return 0;
          }
        }
      }
    }
  }
  return 1;
}

static void place_obs(map* map, point a, point b) {
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
  while (1) {
    if (c.x < 0 || c.x >= map->width || c.y < 0 || c.y >= map->height) {
      break;
    }
    map->points[c.x + c.y * map->width]->is_obstacle = 1;
    if (c.x == b.x && c.y == b.y) {
      break;
    }
    c.x += dx;
    c.y += dy;
  }
}

static double heuristic(point a, point b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

void mark_drone_path(map* map, point p) {
  for (int i = 0; i < map->drone_size; i++) {
    for (int j = 0; j < map->drone_size; j++) {
      int index = (p.x + j) + (p.y + i) * map->width;
      map->points[index]->in_path = 1;
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

  int directions[8][2] = {{0, 1},  {1, 0}, {0, -1},  {-1, 0},
                          {-1, 1}, {1, 1}, {-1, -1}, {1, -1}};

  while (open_set->data) {
    point_on_map* current = stack_pop(&open_set);

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

    for (int i = 0; i < 8; i++) {
      int nx = current->point.x + directions[i][0];
      int ny = current->point.y + directions[i][1];
      point new_point = {nx, ny};

      if (can_place_drone(map, new_point)) {
        point_on_map* neighbor = all_nodes[nx + ny * map->width];

        if (neighbor->is_obstacle || neighbor->last) {
          continue;
        }

        double tentative_g_cost =
            current->g_cost + dist(current->point, neighbor->point);

        if (tentative_g_cost < neighbor->g_cost) {
          neighbor->next = current;
          neighbor->g_cost = tentative_g_cost;
          neighbor->h_cost = heuristic(neighbor->point, goal);
          neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
          neighbor->x_direction = directions[i][0];
          neighbor->y_direction = directions[i][1];

          stack_push(&open_set, neighbor);
        }
      }
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
