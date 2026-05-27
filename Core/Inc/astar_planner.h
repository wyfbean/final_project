#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "mapping_grid.h"

#define ASTAR_PLANNER_MAX_PATH_CELLS 512U

typedef struct
{
  uint8_t x;
  uint8_t y;
} AstarPlannerCell_t;

typedef enum
{
  ASTAR_PLANNER_STATUS_OK = 0,
  ASTAR_PLANNER_STATUS_BAD_ARGUMENT,
  ASTAR_PLANNER_STATUS_BAD_START,
  ASTAR_PLANNER_STATUS_NO_FRONTIER,
  ASTAR_PLANNER_STATUS_NO_PATH,
  ASTAR_PLANNER_STATUS_PATH_TRUNCATED
} AstarPlannerStatus_t;

typedef struct
{
  AstarPlannerCell_t cells[ASTAR_PLANNER_MAX_PATH_CELLS];
  AstarPlannerCell_t target;
  uint16_t length;
  uint16_t expanded_nodes;
  AstarPlannerStatus_t status;
} AstarPlannerPath_t;

AstarPlannerStatus_t AstarPlanner_PlanToFrontier(const MappingGridSnapshot_t *snapshot,
                                                 uint8_t start_x,
                                                 uint8_t start_y,
                                                 AstarPlannerPath_t *out_path);
AstarPlannerStatus_t AstarPlanner_PlanToGoal(const MappingGridSnapshot_t *snapshot,
                                             uint8_t start_x,
                                             uint8_t start_y,
                                             uint8_t goal_x,
                                             uint8_t goal_y,
                                             AstarPlannerPath_t *out_path);
const char *AstarPlanner_StatusName(AstarPlannerStatus_t status);

#endif /* ASTAR_PLANNER_H */
