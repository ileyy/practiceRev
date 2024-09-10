#include "Var_str.h"

stack* stack_new() { return NULL; }

int isEmpty(stack* s) { return s == NULL; }

void stack_push(stack** s, point_on_map* data) {
  stack* newNode = (stack*)malloc(sizeof(stack));
  if (newNode == NULL) {
    printf("Error allocating memmory.\n");
    exit(EXIT_FAILURE);
  }
  newNode->data = data;
  newNode->next = *s;
  *s = newNode;
}

point_on_map* stack_pop(stack** s) {
  if (isEmpty(*s)) {
    printf("Empty stack.\n");
    exit(EXIT_FAILURE);
  }
  stack* temp = *s;
  point_on_map* data = temp->data;
  *s = (*s)->next;
  free(temp);
  return data;
}

int imin(int a, int b) { return a < b ? a : b; }

int imax(int a, int b) { return a > b ? a : b; }

double dist(point a, point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double distance(int x1, int y1, int x2, int y2, int x3, int y3, int x4,
                int y4) {
  double minDist = INFINITY;

  for (int i = 0; i <= 100; i++) {
    double t = i / 100.0;
    double xA = x1 + t * (x2 - x1);
    double yA = y1 + t * (y2 - y1);

    for (int j = 0; j <= 100; j++) {
      double u = j / 100.0;
      double xB = x3 + u * (x4 - x3);
      double yB = y3 + u * (y4 - y3);

      double dist = sqrt(pow(xA - xB, 2) + pow(yA - yB, 2));
      if (dist < minDist) {
        minDist = dist;
      }
    }
  }
  return minDist;
}

void orderPoints(rectangle* obs, int n) {
  for (int i = 0; i < n; i++) {
    if (obs[i].b.x < obs[i].a.x) {
      int temp = obs[i].a.x;
      obs[i].a.x = obs[i].b.x;
      obs[i].b.x = temp;
      temp = obs[i].a.y;
      obs[i].a.y = obs[i].b.y;
      obs[i].b.y = temp;
    }
  }
}

int checkObstacle(int x, int y, rectangle obstacles[], int n) {
  for (int i = 0; i < n; i++) {
    int minX = imin(obstacles[i].a.x, obstacles[i].b.x);
    int maxX = imax(obstacles[i].a.x, obstacles[i].b.x);
    int minY = imin(obstacles[i].a.y, obstacles[i].b.y);
    int maxY = imax(obstacles[i].a.y, obstacles[i].b.y);
    if ((x >= minX && x <= maxX) && (y >= minY && y <= maxY)) {
      return 1;
    }
  }
  return 0;
}

int checkIntersecrion(rectangle newObstacle, rectangle obstacles[], int n) {
  for (int i = 0; i < n; i++) {
    int o1 = ((obstacles[i].a.y - newObstacle.a.y) *
              (newObstacle.b.x - newObstacle.a.x)) -
             ((obstacles[i].a.x - newObstacle.a.x) *
              (newObstacle.b.y - newObstacle.a.y));
    int o2 = ((obstacles[i].b.y - newObstacle.a.y) *
              (newObstacle.b.x - newObstacle.a.x)) -
             ((obstacles[i].b.x - newObstacle.a.x) *
              (newObstacle.b.y - newObstacle.a.y));
    int o3 = ((newObstacle.a.y - obstacles[i].a.y) *
              (obstacles[i].b.x - obstacles[i].a.x)) -
             ((newObstacle.a.x - obstacles[i].a.x) *
              (obstacles[i].b.y - obstacles[i].a.y));
    int o4 = ((newObstacle.b.y - obstacles[i].a.y) *
              (obstacles[i].b.x - obstacles[i].a.x)) -
             ((newObstacle.b.x - obstacles[i].a.x) *
              (obstacles[i].b.y - obstacles[i].a.y));

    if ((o1 * o2 <= 0) && (o3 * o4 <= 0)) {
      if (!((newObstacle.a.x == obstacles[i].a.x &&
             newObstacle.a.y == obstacles[i].a.y) ||
            (newObstacle.a.x == obstacles[i].b.x &&
             newObstacle.a.y == obstacles[i].b.y) ||
            (newObstacle.b.x == obstacles[i].a.x &&
             newObstacle.b.y == obstacles[i].a.y) ||
            (newObstacle.b.x == obstacles[i].b.x &&
             newObstacle.b.y == obstacles[i].b.y))) {
        if (!checkObstacle(newObstacle.a.x, newObstacle.a.y, obstacles, n) &&
            !checkObstacle(newObstacle.b.x, newObstacle.b.y, obstacles, n)) {
          return 1;  // intersect
        }
      }
    }
  }
  return 0;  // dont
}

int checkDistance(rectangle newObstacle, rectangle* obstacles, int n, int x) {
  for (int i = 0; i < n; i++) {
    double minDist = distance(
        newObstacle.a.x, newObstacle.a.y, newObstacle.b.x, newObstacle.b.y,
        obstacles[i].a.x, obstacles[i].a.y, obstacles[i].b.x, obstacles[i].b.y);

    if (minDist < x * sqrt(2)) {
      return 0;
    }
  }
  return 1;
}

void map_clear(map* map) {
  for (int y = 0; y < map->height; y++) {
    for (int x = 0; x < map->width; x++) {
      *map->points[x + y * map->width] =
          (point_on_map){.point = {.x = x, .y = y},
                         .is_obstacle = 0,
                         .in_path = 0,
                         .next = NULL};
    }
  }
}

map* map_new(unsigned width, unsigned height, unsigned droneSize,
             point startPoint) {
  map* map = malloc(sizeof *map);
  map->points = malloc(sizeof *map->points * width * height);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      map->points[x + y * width] = malloc(sizeof **map->points);
    }
  }
  map->start_point = map->points[startPoint.x + startPoint.y * width];
  map->width = width;
  map->height = height;
  map->drone_size = droneSize;
  map_clear(map);
  return map;
}
