#include "Var_str.h"

int generateObstacles(int droneSize, int width, int height, int numObstacles,
                      rectangle obstacles[]);

int main1() {
  int width, height, droneSize, numObstacles;
  int result = 0;
  printf("Enter the drone size (length of one side): ");
  scanf("%d", &droneSize);

  printf("Enter the map width and height: ");
  scanf("%d %d", &width, &height);
  if (!(droneSize < width && droneSize < height)) {
    do {
      printf("Map size must be greater than drone size. Try again: ");
      scanf("%d %d", &width, &height);
    } while (!(droneSize < width && droneSize < height));
  }

  while (!result) {
    printf("Enter number of obstacles: ");
    scanf("%d", &numObstacles);
    rectangle *obstacles = malloc(numObstacles * sizeof(rectangle));
    if (obstacles == NULL) {
      fprintf(stderr, "Error allocating memmory.\n");
      return 0;
    }
    result =
        generateObstacles(droneSize, width, height, numObstacles, obstacles);
    if (!result) {
      printf("Error.\n");
      free(obstacles);
    } else {
      FILE *map = fopen("map.txt", "w");
      if (map == NULL) {
        fprintf(stderr, "Error openning file.\n");
        return 1;
      }
      fprintf(map, "Map size(W x H): %d x %d\n", width, height);
      fprintf(map, "Drone size: %d\n", droneSize);
      fprintf(map, "Number of obstacles: %d\n", numObstacles);
      for (int i = 0; i < numObstacles; i++) {
        orderPoints(&obstacles[i], 1);
        fprintf(map, "Obstacle coordinates: (%d, %d) (%d, %d)\n",
                obstacles[i].a.x, obstacles[i].a.y, obstacles[i].b.x,
                obstacles[i].b.y);
      }
      fprintf(map, "Start point: (, )\n");
      fprintf(map, "End point: (, )\n");
      fclose(map);
      free(obstacles);
      printf(
          "Generating done. Please enter start and end points at the end of "
          "the map.txt file.\n");
    }
  }
  return 0;
}

int generateObstacles(int droneSize, int width, int height, int numObstacles,
                      rectangle obstacles[]) {
  srand(time(NULL));
  for (int i = 0; i < numObstacles;) {
    int attempts = 0;
    while (attempts < numObstacles * width) {
      rectangle newObstacle;
      int type = rand() % 3;
      newObstacle.a.x = rand() % width;
      newObstacle.a.y = rand() % height;
      if (type == 0) {  // horizontal
        newObstacle.b.x = newObstacle.a.x + rand() % (width - newObstacle.a.x);
        newObstacle.b.y = newObstacle.a.y;
      } else if (type == 1) {  // vertical
        newObstacle.b.x = newObstacle.a.x;
        newObstacle.b.y = newObstacle.a.y + rand() % (height - newObstacle.a.y);
      } else {  // diagonal
        int direction = rand() % 4;
        if (direction == 0) {  // -1 1
          int delta = rand() % (imin(newObstacle.a.x, newObstacle.a.y) + 1);
          newObstacle.b.x = newObstacle.a.x - delta;
          newObstacle.b.y = newObstacle.a.y - delta;
        } else if (direction == 1) {  // 1 -1
          int delta =
              rand() %
              (imin(width - newObstacle.a.x, height - newObstacle.a.y) + 1);
          newObstacle.b.x = newObstacle.a.x + delta;
          newObstacle.b.y = newObstacle.a.y + delta;
        } else if (direction == 2) {  // -1 -1
          int delta =
              rand() % (imin(newObstacle.a.x, height - newObstacle.a.y) + 1);
          newObstacle.b.x = newObstacle.a.x - delta;
          newObstacle.b.y = newObstacle.a.y + delta;
        } else if (direction == 3) {  // 1 1
          int delta =
              rand() % (imin(width - newObstacle.a.x, newObstacle.a.y) + 1);
          newObstacle.b.x = newObstacle.a.x + delta;
          newObstacle.b.y = newObstacle.a.y - delta;
        }
      }
      if (newObstacle.b.x >= 0 && newObstacle.b.x < width - 1 &&
          newObstacle.b.y >= 0 && newObstacle.b.y < height - 1 &&
          !(newObstacle.a.x == newObstacle.b.x &&
            newObstacle.a.y == newObstacle.b.y)) {
        if (!checkIntersecrion(newObstacle, obstacles, i) &&
            checkDistance(newObstacle, obstacles, i, droneSize) &&
            !checkObstacle(newObstacle.a.x, newObstacle.a.y, obstacles, i) &&
            !checkObstacle(newObstacle.b.x, newObstacle.b.y, obstacles, i)) {
          if (sqrt((pow(newObstacle.b.x - newObstacle.a.x, 2)) +
                   (pow(newObstacle.b.y - newObstacle.a.y, 2))) <=
              imin(width, height) / 2) {
            obstacles[i++] = newObstacle;
            break;
          }
        }
      }
      attempts++;
    }
    if (attempts >= numObstacles * width) {
      printf("Error generating obstacles.\n");
      return 0;  // error
    }
  }
  return 1;  // done
}
