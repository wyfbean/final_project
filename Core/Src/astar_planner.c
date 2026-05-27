#include "astar_planner.h"

#include <string.h>

#define ASTAR_TOTAL_CELLS ((uint16_t)(MAPPING_GRID_WIDTH_CELLS * MAPPING_GRID_HEIGHT_CELLS))
#define ASTAR_FRONTIER_CANDIDATE_LIMIT 64U
#define ASTAR_OBSTACLE_INFLATION_CELLS 0
#define ASTAR_FREE_STEP_COST 1U
#define ASTAR_HEURISTIC_WEIGHT 1U
#define ASTAR_NODE_FLAG_OPEN 0x01U
#define ASTAR_NODE_FLAG_CLOSED 0x02U
#define ASTAR_COST_INF 0xFFFFU

typedef struct
{
  uint16_t index;
  uint16_t distance;
} AstarFrontierCandidate_t;

static uint16_t s_g_score[ASTAR_TOTAL_CELLS];
static uint16_t s_parent[ASTAR_TOTAL_CELLS];
static uint8_t s_node_flags[ASTAR_TOTAL_CELLS];
static AstarFrontierCandidate_t s_frontier_candidates[ASTAR_FRONTIER_CANDIDATE_LIMIT];

static uint16_t Astar_Index(uint8_t x, uint8_t y);
static void Astar_Cell(uint16_t index, uint8_t *out_x, uint8_t *out_y);
static uint16_t Astar_Manhattan(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by);
static bool Astar_IsInside(int16_t x, int16_t y);
static bool Astar_IsFree(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y);
static bool Astar_IsSearchPassable(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y);
static bool Astar_IsInflatedPassable(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y);
static bool Astar_IsFrontierCandidate(const MappingGridSnapshot_t *snapshot,
                                      uint8_t start_x,
                                      uint8_t start_y,
                                      uint8_t x,
                                      uint8_t y);
static AstarPlannerStatus_t Astar_PlanToFrontierInternal(const MappingGridSnapshot_t *snapshot,
                                                         uint8_t start_x,
                                                         uint8_t start_y,
                                                         AstarPlannerPath_t *out_path);
static uint8_t Astar_CollectFrontierCandidates(const MappingGridSnapshot_t *snapshot,
                                               uint8_t start_x,
                                               uint8_t start_y);
static bool Astar_CandidateBetter(uint16_t distance, uint16_t other_distance);
static void Astar_InsertCandidate(uint16_t index, uint16_t distance, uint8_t *count);
static AstarPlannerStatus_t Astar_SearchToGoal(const MappingGridSnapshot_t *snapshot,
                                               uint8_t start_x,
                                               uint8_t start_y,
                                               uint8_t goal_x,
                                               uint8_t goal_y,
                                               AstarPlannerPath_t *out_path);
static bool Astar_ReconstructPath(uint16_t start_index,
                                  uint16_t goal_index,
                                  AstarPlannerPath_t *out_path);
static void Astar_ClearSearch(void);
static uint16_t Astar_PickBestOpen(uint8_t goal_x, uint8_t goal_y);

AstarPlannerStatus_t AstarPlanner_PlanToFrontier(const MappingGridSnapshot_t *snapshot,
                                                 uint8_t start_x,
                                                 uint8_t start_y,
                                                 AstarPlannerPath_t *out_path)
{
  return Astar_PlanToFrontierInternal(snapshot, start_x, start_y, out_path);
}

static AstarPlannerStatus_t Astar_PlanToFrontierInternal(const MappingGridSnapshot_t *snapshot,
                                                         uint8_t start_x,
                                                         uint8_t start_y,
                                                         AstarPlannerPath_t *out_path)
{
  uint8_t candidate_count;
  uint8_t i;

  if ((snapshot == NULL) || (out_path == NULL))
  {
    return ASTAR_PLANNER_STATUS_BAD_ARGUMENT;
  }

  memset(out_path, 0, sizeof(*out_path));
  out_path->status = ASTAR_PLANNER_STATUS_NO_PATH;

  if ((start_x >= MAPPING_GRID_WIDTH_CELLS) ||
      (start_y >= MAPPING_GRID_HEIGHT_CELLS) ||
      !Astar_IsFree(snapshot, start_x, start_y))
  {
    out_path->status = ASTAR_PLANNER_STATUS_BAD_START;
    return out_path->status;
  }

  candidate_count = Astar_CollectFrontierCandidates(
      snapshot,
      start_x,
      start_y);
  if (candidate_count == 0U)
  {
    out_path->status = ASTAR_PLANNER_STATUS_NO_FRONTIER;
    return out_path->status;
  }

  for (i = 0U; i < candidate_count; ++i)
  {
    uint8_t goal_x;
    uint8_t goal_y;
    AstarPlannerStatus_t status;

    Astar_Cell(s_frontier_candidates[i].index, &goal_x, &goal_y);
    status = Astar_SearchToGoal(snapshot, start_x, start_y, goal_x, goal_y, out_path);
    if ((status == ASTAR_PLANNER_STATUS_OK) ||
        (status == ASTAR_PLANNER_STATUS_PATH_TRUNCATED))
    {
      out_path->target.x = goal_x;
      out_path->target.y = goal_y;
      out_path->status = status;
      return status;
    }
  }

  out_path->status = ASTAR_PLANNER_STATUS_NO_PATH;
  return out_path->status;
}

AstarPlannerStatus_t AstarPlanner_PlanToGoal(const MappingGridSnapshot_t *snapshot,
                                             uint8_t start_x,
                                             uint8_t start_y,
                                             uint8_t goal_x,
                                             uint8_t goal_y,
                                             AstarPlannerPath_t *out_path)
{
  AstarPlannerStatus_t status;

  if ((snapshot == NULL) || (out_path == NULL))
  {
    return ASTAR_PLANNER_STATUS_BAD_ARGUMENT;
  }

  memset(out_path, 0, sizeof(*out_path));
  out_path->status = ASTAR_PLANNER_STATUS_NO_PATH;

  if ((start_x >= MAPPING_GRID_WIDTH_CELLS) ||
      (start_y >= MAPPING_GRID_HEIGHT_CELLS) ||
      !Astar_IsFree(snapshot, start_x, start_y))
  {
    out_path->status = ASTAR_PLANNER_STATUS_BAD_START;
    return out_path->status;
  }

  if ((goal_x >= MAPPING_GRID_WIDTH_CELLS) ||
      (goal_y >= MAPPING_GRID_HEIGHT_CELLS) ||
      !Astar_IsSearchPassable(snapshot, goal_x, goal_y))
  {
    out_path->status = ASTAR_PLANNER_STATUS_NO_PATH;
    return out_path->status;
  }

  status = Astar_SearchToGoal(snapshot, start_x, start_y, goal_x, goal_y, out_path);
  out_path->target.x = goal_x;
  out_path->target.y = goal_y;
  out_path->status = status;
  return status;
}

const char *AstarPlanner_StatusName(AstarPlannerStatus_t status)
{
  switch (status)
  {
    case ASTAR_PLANNER_STATUS_OK:             return "OK";
    case ASTAR_PLANNER_STATUS_BAD_ARGUMENT:   return "BAD_ARG";
    case ASTAR_PLANNER_STATUS_BAD_START:      return "BAD_START";
    case ASTAR_PLANNER_STATUS_NO_FRONTIER:    return "NO_FRONTIER";
    case ASTAR_PLANNER_STATUS_NO_PATH:        return "NO_PATH";
    case ASTAR_PLANNER_STATUS_PATH_TRUNCATED: return "TRUNCATED";
    default:                                  return "UNKNOWN";
  }
}

static uint16_t Astar_Index(uint8_t x, uint8_t y)
{
  return (uint16_t)(((uint16_t)y * (uint16_t)MAPPING_GRID_WIDTH_CELLS) + (uint16_t)x);
}

static void Astar_Cell(uint16_t index, uint8_t *out_x, uint8_t *out_y)
{
  if (out_x != NULL)
  {
    *out_x = (uint8_t)(index % (uint16_t)MAPPING_GRID_WIDTH_CELLS);
  }

  if (out_y != NULL)
  {
    *out_y = (uint8_t)(index / (uint16_t)MAPPING_GRID_WIDTH_CELLS);
  }
}

static uint16_t Astar_Manhattan(uint8_t ax, uint8_t ay, uint8_t bx, uint8_t by)
{
  uint16_t dx = (ax >= bx) ? (uint16_t)(ax - bx) : (uint16_t)(bx - ax);
  uint16_t dy = (ay >= by) ? (uint16_t)(ay - by) : (uint16_t)(by - ay);
  return (uint16_t)(dx + dy);
}

static bool Astar_IsInside(int16_t x, int16_t y)
{
  return ((x >= 0) &&
          (y >= 0) &&
          (x < (int16_t)MAPPING_GRID_WIDTH_CELLS) &&
          (y < (int16_t)MAPPING_GRID_HEIGHT_CELLS));
}

static bool Astar_IsFree(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y)
{
  if ((snapshot == NULL) ||
      (x >= MAPPING_GRID_WIDTH_CELLS) ||
      (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return false;
  }

  return snapshot->cells[y][x] == MAPPING_GRID_CELL_FREE;
}

static bool Astar_IsSearchPassable(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y)
{
  if ((snapshot == NULL) ||
      (x >= MAPPING_GRID_WIDTH_CELLS) ||
      (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return false;
  }

  return snapshot->cells[y][x] == MAPPING_GRID_CELL_FREE;
}

static bool Astar_IsInflatedPassable(const MappingGridSnapshot_t *snapshot, uint8_t x, uint8_t y)
{
#if ASTAR_OBSTACLE_INFLATION_CELLS > 0
  int16_t dx;
  int16_t dy;
#endif

  if (!Astar_IsFree(snapshot, x, y))
  {
    return false;
  }

#if ASTAR_OBSTACLE_INFLATION_CELLS > 0
  for (dy = -ASTAR_OBSTACLE_INFLATION_CELLS; dy <= ASTAR_OBSTACLE_INFLATION_CELLS; ++dy)
  {
    for (dx = -ASTAR_OBSTACLE_INFLATION_CELLS; dx <= ASTAR_OBSTACLE_INFLATION_CELLS; ++dx)
    {
      int16_t nx = (int16_t)x + dx;
      int16_t ny = (int16_t)y + dy;

      if (!Astar_IsInside(nx, ny))
      {
        return false;
      }

      if (snapshot->cells[ny][nx] == MAPPING_GRID_CELL_OCCUPIED)
      {
        return false;
      }
    }
  }
#endif

  return true;
}

static bool Astar_IsFrontierCandidate(const MappingGridSnapshot_t *snapshot,
                                      uint8_t start_x,
                                      uint8_t start_y,
                                      uint8_t x,
                                      uint8_t y)
{
  static const int8_t offsets[4][2] = {
      {1, 0},
      {-1, 0},
      {0, 1},
      {0, -1},
  };
  uint8_t i;

  if (((x == start_x) && (y == start_y)) ||
      !Astar_IsInflatedPassable(snapshot, x, y))
  {
    return false;
  }

  for (i = 0U; i < 4U; ++i)
  {
    int16_t nx = (int16_t)x + offsets[i][0];
    int16_t ny = (int16_t)y + offsets[i][1];

    if (Astar_IsInside(nx, ny) &&
        (snapshot->cells[ny][nx] == MAPPING_GRID_CELL_UNKNOWN))
    {
      return true;
    }
  }

  return false;
}

static uint8_t Astar_CollectFrontierCandidates(const MappingGridSnapshot_t *snapshot,
                                               uint8_t start_x,
                                               uint8_t start_y)
{
  uint8_t count = 0U;
  uint8_t x;
  uint8_t y;

  for (y = 0U; y < MAPPING_GRID_HEIGHT_CELLS; ++y)
  {
    for (x = 0U; x < MAPPING_GRID_WIDTH_CELLS; ++x)
    {
      if (Astar_IsFrontierCandidate(snapshot, start_x, start_y, x, y))
      {
        Astar_InsertCandidate(
            Astar_Index(x, y),
            Astar_Manhattan(start_x, start_y, x, y),
            &count);
      }
    }
  }

  return count;
}

static bool Astar_CandidateBetter(uint16_t distance, uint16_t other_distance)
{
  return distance < other_distance;
}

static void Astar_InsertCandidate(uint16_t index, uint16_t distance, uint8_t *count)
{
  uint8_t pos;

  if (count == NULL)
  {
    return;
  }

  pos = *count;
  if (pos >= ASTAR_FRONTIER_CANDIDATE_LIMIT)
  {
    if (!Astar_CandidateBetter(
            distance,
            s_frontier_candidates[ASTAR_FRONTIER_CANDIDATE_LIMIT - 1U].distance))
    {
      return;
    }
    pos = ASTAR_FRONTIER_CANDIDATE_LIMIT - 1U;
  }
  else
  {
    (*count)++;
  }

  while ((pos > 0U) &&
         Astar_CandidateBetter(
             distance,
             s_frontier_candidates[pos - 1U].distance))
  {
    s_frontier_candidates[pos] = s_frontier_candidates[pos - 1U];
    pos--;
  }

  s_frontier_candidates[pos].index = index;
  s_frontier_candidates[pos].distance = distance;
}

static AstarPlannerStatus_t Astar_SearchToGoal(const MappingGridSnapshot_t *snapshot,
                                               uint8_t start_x,
                                               uint8_t start_y,
                                               uint8_t goal_x,
                                               uint8_t goal_y,
                                               AstarPlannerPath_t *out_path)
{
  static const int8_t offsets[4][2] = {
      {1, 0},
      {-1, 0},
      {0, 1},
      {0, -1},
  };
  uint16_t start_index = Astar_Index(start_x, start_y);
  uint16_t goal_index = Astar_Index(goal_x, goal_y);
  uint16_t expanded = 0U;

  Astar_ClearSearch();
  s_g_score[start_index] = 0U;
  s_parent[start_index] = start_index;
  s_node_flags[start_index] = ASTAR_NODE_FLAG_OPEN;

  for (;;)
  {
    uint16_t current = Astar_PickBestOpen(goal_x, goal_y);
    uint8_t current_x;
    uint8_t current_y;
    uint8_t i;

    if (current == ASTAR_COST_INF)
    {
      out_path->expanded_nodes = expanded;
      return ASTAR_PLANNER_STATUS_NO_PATH;
    }

    if (current == goal_index)
    {
      out_path->expanded_nodes = expanded;
      return Astar_ReconstructPath(start_index, goal_index, out_path) ?
          out_path->status :
          ASTAR_PLANNER_STATUS_NO_PATH;
    }

    s_node_flags[current] &= (uint8_t)(~ASTAR_NODE_FLAG_OPEN);
    s_node_flags[current] |= ASTAR_NODE_FLAG_CLOSED;
    expanded++;
    Astar_Cell(current, &current_x, &current_y);

    for (i = 0U; i < 4U; ++i)
    {
      int16_t nx = (int16_t)current_x + offsets[i][0];
      int16_t ny = (int16_t)current_y + offsets[i][1];
      uint16_t neighbor;
      uint16_t tentative_g;

      if (!Astar_IsInside(nx, ny))
      {
        continue;
      }

      neighbor = Astar_Index((uint8_t)nx, (uint8_t)ny);
      if ((s_node_flags[neighbor] & ASTAR_NODE_FLAG_CLOSED) != 0U)
      {
        continue;
      }

      if (!Astar_IsSearchPassable(snapshot, (uint8_t)nx, (uint8_t)ny))
      {
        continue;
      }

      tentative_g = (uint16_t)(s_g_score[current] + ASTAR_FREE_STEP_COST);
      if (((s_node_flags[neighbor] & ASTAR_NODE_FLAG_OPEN) == 0U) ||
          (tentative_g < s_g_score[neighbor]))
      {
        s_parent[neighbor] = current;
        s_g_score[neighbor] = tentative_g;
        s_node_flags[neighbor] |= ASTAR_NODE_FLAG_OPEN;
      }
    }
  }
}

static bool Astar_ReconstructPath(uint16_t start_index,
                                  uint16_t goal_index,
                                  AstarPlannerPath_t *out_path)
{
  uint16_t full_length = 1U;
  uint16_t node = goal_index;
  uint16_t output_length;
  uint16_t i;

  while (node != start_index)
  {
    node = s_parent[node];
    full_length++;
    if (full_length > ASTAR_TOTAL_CELLS)
    {
      return false;
    }
  }

  output_length = (full_length > ASTAR_PLANNER_MAX_PATH_CELLS) ?
      ASTAR_PLANNER_MAX_PATH_CELLS :
      full_length;

  for (i = 0U; i < output_length; ++i)
  {
    uint16_t steps_from_goal = (uint16_t)(full_length - 1U - i);
    uint16_t path_node = goal_index;
    uint16_t step;

    for (step = 0U; step < steps_from_goal; ++step)
    {
      path_node = s_parent[path_node];
    }

    Astar_Cell(path_node, &out_path->cells[i].x, &out_path->cells[i].y);
  }

  out_path->length = output_length;
  out_path->status = (full_length > ASTAR_PLANNER_MAX_PATH_CELLS) ?
      ASTAR_PLANNER_STATUS_PATH_TRUNCATED :
      ASTAR_PLANNER_STATUS_OK;
  return true;
}

static void Astar_ClearSearch(void)
{
  uint16_t i;

  for (i = 0U; i < ASTAR_TOTAL_CELLS; ++i)
  {
    s_g_score[i] = ASTAR_COST_INF;
    s_parent[i] = ASTAR_COST_INF;
    s_node_flags[i] = 0U;
  }
}

static uint16_t Astar_PickBestOpen(uint8_t goal_x, uint8_t goal_y)
{
  uint16_t best_index = ASTAR_COST_INF;
  uint16_t best_f = ASTAR_COST_INF;
  uint16_t best_h = ASTAR_COST_INF;
  uint16_t i;

  for (i = 0U; i < ASTAR_TOTAL_CELLS; ++i)
  {
    if ((s_node_flags[i] & ASTAR_NODE_FLAG_OPEN) != 0U)
    {
      uint8_t x;
      uint8_t y;
      uint16_t h;
      uint16_t weighted_h;
      uint16_t f;

      Astar_Cell(i, &x, &y);
      h = Astar_Manhattan(x, y, goal_x, goal_y);
      weighted_h = (uint16_t)(h * ASTAR_HEURISTIC_WEIGHT);
      f = (uint16_t)(s_g_score[i] + weighted_h);
      if ((best_index == ASTAR_COST_INF) ||
          (f < best_f) ||
          ((f == best_f) && (h < best_h)))
      {
        best_index = i;
        best_f = f;
        best_h = h;
      }
    }
  }

  return best_index;
}
