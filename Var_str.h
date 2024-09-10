#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct {
  int x, y;  // coordinates
} point;

typedef struct point_on_map {
  point point;
  int is_obstacle;
  int in_path;
  int last;
  int x_direction;
  int y_direction;
  double g_cost;  // cost from start to current point
  double h_cost;  // heuristic cost from current point to goal
  double f_cost;  // total cost (g_cost + h_cost)
  struct point_on_map* next;
} point_on_map;

typedef struct {
  point_on_map** points;
  point_on_map* start_point;
  unsigned width, height, drone_size;
} map;

typedef struct {
  point a, b;
} rectangle;

typedef struct stack {
  point_on_map* data;
  struct stack* next;
} stack;

stack* stack_new();
int isEmpty(stack* s);
void stack_push(stack** s, point_on_map* data);
point_on_map* stack_pop(stack** s);
int imin(int a, int b);
int imax(int a, int b);
double dist(point a, point b);
double distance(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
void orderPoints(rectangle* obs, int n);
int checkObstacle(int x, int y, rectangle obstacles[], int n);
int checkIntersecrion(rectangle newObstacle, rectangle obstacles[], int n);
int checkDistance(rectangle newObstacle, rectangle* obstacles, int n, int x);
void map_clear(map* map);
map* map_new(unsigned width, unsigned height, unsigned droneSize,
             point startPoint);
