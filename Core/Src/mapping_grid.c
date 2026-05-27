#include "mapping_grid.h"

#include "FreeRTOS.h"
#include "lidar_pipeline.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define MAPPING_GRID_TOTAL_CELLS       (MAPPING_GRID_WIDTH_CELLS * MAPPING_GRID_HEIGHT_CELLS)
#define MAPPING_GRID_HALF_WIDTH_MM     ((int32_t)((MAPPING_GRID_WIDTH_CELLS * MAPPING_GRID_CELL_SIZE_MM) / 2U))
#define MAPPING_GRID_HALF_HEIGHT_MM    ((int32_t)((MAPPING_GRID_HEIGHT_CELLS * MAPPING_GRID_CELL_SIZE_MM) / 2U))
#define MAPPING_GRID_FREE_DELTA        3
#define MAPPING_GRID_FREE_POSITIVE_DELTA 2
#define MAPPING_GRID_FREE_OCCUPIED_DELTA 3
#define MAPPING_GRID_OCCUPIED_DELTA    6
#define MAPPING_GRID_NEAR_OCCUPIED_DELTA 3
#define MAPPING_GRID_MIN_SCORE         (-40)
#define MAPPING_GRID_MAX_SCORE         30
#define MAPPING_GRID_FREE_THRESHOLD    (-6)
#define MAPPING_GRID_OCCUPIED_THRESHOLD 18
#define MAPPING_GRID_MIN_DISTANCE_MM   175U
#define MAPPING_GRID_NEAR_WEAK_DISTANCE_MM (MAPPING_GRID_CELL_SIZE_MM * 3U)
#define MAPPING_GRID_MIN_QUALITY       5U
#define MAPPING_GRID_PI                3.14159265358979323846f

static int8_t s_grid[MAPPING_GRID_HEIGHT_CELLS][MAPPING_GRID_WIDTH_CELLS];
static MappingGridPose_t s_pose;
static bool s_pose_valid;
static MappingGridStats_t s_stats;

static int32_t MappingGrid_NormalizeAngleCdeg(int32_t angle_cdeg);
static float MappingGrid_CdegToRadians(int32_t angle_cdeg);
static bool MappingGrid_InsertPolarPointWithPose(const MappingGridPose_t *pose,
                                                 uint16_t angle_cdeg,
                                                 uint16_t distance_mm,
                                                 uint8_t quality);
static void MappingGrid_MarkFree(uint8_t x, uint8_t y);
static void MappingGrid_MarkOccupied(uint8_t x, uint8_t y, uint16_t distance_mm);
static void MappingGrid_UpdateScore(uint8_t x, uint8_t y, int8_t delta);
static void MappingGrid_ForceFree(uint8_t x, uint8_t y);
static void MappingGrid_ApplyStateChange(MappingGridCellState_t old_state, MappingGridCellState_t new_state);
static void MappingGrid_TraceFreeRay(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y);
static bool MappingGrid_FindClippedRayEnd(int32_t start_x_mm,
                                          int32_t start_y_mm,
                                          int32_t end_x_mm,
                                          int32_t end_y_mm,
                                          uint8_t *out_x,
                                          uint8_t *out_y);
static char MappingGrid_CellToChar(MappingGridCellState_t state);

void MappingGrid_Init(void)
{
  MappingGrid_Reset();
}

void MappingGrid_Reset(void)
{
  taskENTER_CRITICAL();
  memset(s_grid, 0, sizeof(s_grid));
  memset(&s_stats, 0, sizeof(s_stats));
  memset(&s_pose, 0, sizeof(s_pose));
  s_pose_valid = false;
  s_stats.unknown_cells = (uint16_t)MAPPING_GRID_TOTAL_CELLS;
  taskEXIT_CRITICAL();
}

void MappingGrid_SetPose(const MappingGridPose_t *pose)
{
  if (pose == NULL)
  {
    taskENTER_CRITICAL();
    s_pose_valid = false;
    taskEXIT_CRITICAL();
    return;
  }

  taskENTER_CRITICAL();
  s_pose = *pose;
  s_pose.heading_cdeg = MappingGrid_NormalizeAngleCdeg(s_pose.heading_cdeg);
  s_pose_valid = true;
  taskEXIT_CRITICAL();
}

bool MappingGrid_GetPose(MappingGridPose_t *out_pose)
{
  if ((out_pose == NULL) || !s_pose_valid)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_pose = s_pose;
  taskEXIT_CRITICAL();
  return true;
}

bool MappingGrid_InsertPolarPoint(uint16_t angle_cdeg, uint16_t distance_mm, uint8_t quality)
{
  if (!s_pose_valid)
  {
    s_stats.rejected_points++;
    return false;
  }

  return MappingGrid_InsertPolarPointWithPose(&s_pose, angle_cdeg, distance_mm, quality);
}

bool MappingGrid_InsertPolarPointAtPose(const MappingGridPose_t *pose,
                                        uint16_t angle_cdeg,
                                        uint16_t distance_mm,
                                        uint8_t quality)
{
  if (pose == NULL)
  {
    s_stats.rejected_points++;
    return false;
  }

  return MappingGrid_InsertPolarPointWithPose(pose, angle_cdeg, distance_mm, quality);
}

static bool MappingGrid_InsertPolarPointWithPose(const MappingGridPose_t *pose,
                                                 uint16_t angle_cdeg,
                                                 uint16_t distance_mm,
                                                 uint8_t quality)
{
  uint8_t robot_x;
  uint8_t robot_y;
  uint8_t hit_x;
  uint8_t hit_y;
  int32_t world_angle_cdeg;
  int32_t hit_world_x_mm;
  int32_t hit_world_y_mm;
  float angle_rad;

  if ((quality < MAPPING_GRID_MIN_QUALITY) ||
      (distance_mm < MAPPING_GRID_MIN_DISTANCE_MM) ||
      (distance_mm > MAPPING_GRID_MAX_RANGE_MM))
  {
    s_stats.rejected_points++;
    return false;
  }

  if (!MappingGrid_WorldToCell(pose->x_mm, pose->y_mm, &robot_x, &robot_y))
  {
    s_stats.rejected_points++;
    return false;
  }

  world_angle_cdeg = MappingGrid_NormalizeAngleCdeg(
      pose->heading_cdeg + LidarPipeline_LidarToRobotAngleCdeg(angle_cdeg));
  angle_rad = MappingGrid_CdegToRadians(world_angle_cdeg);
  hit_world_x_mm = pose->x_mm + (int32_t)((float)distance_mm * cosf(angle_rad));
  hit_world_y_mm = pose->y_mm + (int32_t)((float)distance_mm * sinf(angle_rad));

  taskENTER_CRITICAL();
  if (MappingGrid_WorldToCell(hit_world_x_mm, hit_world_y_mm, &hit_x, &hit_y))
  {
    MappingGrid_TraceFreeRay(robot_x, robot_y, hit_x, hit_y);
    if ((hit_x != robot_x) || (hit_y != robot_y))
    {
      MappingGrid_MarkOccupied(hit_x, hit_y, distance_mm);
    }
  }
  else if (MappingGrid_FindClippedRayEnd(pose->x_mm, pose->y_mm, hit_world_x_mm, hit_world_y_mm, &hit_x, &hit_y))
  {
    MappingGrid_TraceFreeRay(robot_x, robot_y, hit_x, hit_y);
    s_stats.clipped_rays++;
  }
  else
  {
    s_stats.clipped_rays++;
    s_stats.rejected_points++;
    taskEXIT_CRITICAL();
    return false;
  }

  s_stats.inserted_points++;
  s_stats.revision++;
  taskEXIT_CRITICAL();
  return true;
}

bool MappingGrid_WorldToCell(int32_t x_mm, int32_t y_mm, uint8_t *out_x, uint8_t *out_y)
{
  int32_t shifted_x = x_mm + MAPPING_GRID_HALF_WIDTH_MM;
  int32_t shifted_y = MAPPING_GRID_HALF_HEIGHT_MM - y_mm;
  int32_t cell_x;
  int32_t cell_y;

  if ((out_x == NULL) || (out_y == NULL) || (shifted_x < 0L) || (shifted_y < 0L))
  {
    return false;
  }

  cell_x = shifted_x / (int32_t)MAPPING_GRID_CELL_SIZE_MM;
  cell_y = shifted_y / (int32_t)MAPPING_GRID_CELL_SIZE_MM;

  if ((cell_x < 0L) ||
      (cell_y < 0L) ||
      (cell_x >= (int32_t)MAPPING_GRID_WIDTH_CELLS) ||
      (cell_y >= (int32_t)MAPPING_GRID_HEIGHT_CELLS))
  {
    return false;
  }

  *out_x = (uint8_t)cell_x;
  *out_y = (uint8_t)cell_y;
  return true;
}

bool MappingGrid_CellToWorld(uint8_t x, uint8_t y, int32_t *out_x_mm, int32_t *out_y_mm)
{
  if ((x >= MAPPING_GRID_WIDTH_CELLS) ||
      (y >= MAPPING_GRID_HEIGHT_CELLS) ||
      (out_x_mm == NULL) ||
      (out_y_mm == NULL))
  {
    return false;
  }

  *out_x_mm = ((int32_t)x * (int32_t)MAPPING_GRID_CELL_SIZE_MM) -
      MAPPING_GRID_HALF_WIDTH_MM +
      ((int32_t)MAPPING_GRID_CELL_SIZE_MM / 2L);
  *out_y_mm = MAPPING_GRID_HALF_HEIGHT_MM -
      ((int32_t)y * (int32_t)MAPPING_GRID_CELL_SIZE_MM) -
      ((int32_t)MAPPING_GRID_CELL_SIZE_MM / 2L);
  return true;
}

MappingGridCellState_t MappingGrid_GetCell(uint8_t x, uint8_t y)
{
  int8_t score;

  if ((x >= MAPPING_GRID_WIDTH_CELLS) || (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return MAPPING_GRID_CELL_UNKNOWN;
  }

  score = s_grid[y][x];
  if (score >= MAPPING_GRID_OCCUPIED_THRESHOLD)
  {
    return MAPPING_GRID_CELL_OCCUPIED;
  }

  if (score <= MAPPING_GRID_FREE_THRESHOLD)
  {
    return MAPPING_GRID_CELL_FREE;
  }

  return MAPPING_GRID_CELL_UNKNOWN;
}

bool MappingGrid_CopySnapshot(MappingGridSnapshot_t *out_snapshot)
{
  uint8_t x;
  uint8_t y;

  if (out_snapshot == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  for (y = 0U; y < MAPPING_GRID_HEIGHT_CELLS; ++y)
  {
    for (x = 0U; x < MAPPING_GRID_WIDTH_CELLS; ++x)
    {
      out_snapshot->cells[y][x] = MappingGrid_GetCell(x, y);
    }
  }
  out_snapshot->pose = s_pose;
  out_snapshot->pose_valid = s_pose_valid;
  out_snapshot->revision = s_stats.revision;
  taskEXIT_CRITICAL();
  return true;
}

void MappingGrid_MarkRobotFree(const MappingGridPose_t *pose, uint8_t radius_cells)
{
  uint8_t center_x;
  uint8_t center_y;
  int16_t dx;
  int16_t dy;

  if ((pose == NULL) ||
      !MappingGrid_WorldToCell(pose->x_mm, pose->y_mm, &center_x, &center_y))
  {
    return;
  }

  taskENTER_CRITICAL();
  for (dy = -((int16_t)radius_cells); dy <= (int16_t)radius_cells; ++dy)
  {
    for (dx = -((int16_t)radius_cells); dx <= (int16_t)radius_cells; ++dx)
    {
      int16_t x = (int16_t)center_x + dx;
      int16_t y = (int16_t)center_y + dy;

      if ((x >= 0) &&
          (y >= 0) &&
          (x < (int16_t)MAPPING_GRID_WIDTH_CELLS) &&
          (y < (int16_t)MAPPING_GRID_HEIGHT_CELLS))
      {
        MappingGrid_ForceFree((uint8_t)x, (uint8_t)y);
      }
    }
  }
  s_stats.revision++;
  taskEXIT_CRITICAL();
}

bool MappingGrid_FormatRow(uint8_t row, char *out_text, uint16_t out_size)
{
  uint8_t x;

  if ((row >= MAPPING_GRID_HEIGHT_CELLS) ||
      (out_text == NULL) ||
      (out_size <= MAPPING_GRID_WIDTH_CELLS))
  {
    return false;
  }

  for (x = 0U; x < MAPPING_GRID_WIDTH_CELLS; ++x)
  {
    out_text[x] = MappingGrid_CellToChar(MappingGrid_GetCell(x, row));
  }
  out_text[MAPPING_GRID_WIDTH_CELLS] = '\0';
  return true;
}

bool MappingGrid_GetStats(MappingGridStats_t *out_stats)
{
  if (out_stats == NULL)
  {
    return false;
  }

  *out_stats = s_stats;
  return true;
}

uint32_t MappingGrid_GetRevision(void)
{
  return s_stats.revision;
}

static int32_t MappingGrid_NormalizeAngleCdeg(int32_t angle_cdeg)
{
  while (angle_cdeg < 0L)
  {
    angle_cdeg += 36000L;
  }

  while (angle_cdeg >= 36000L)
  {
    angle_cdeg -= 36000L;
  }

  return angle_cdeg;
}

static float MappingGrid_CdegToRadians(int32_t angle_cdeg)
{
  return ((float)angle_cdeg * MAPPING_GRID_PI) / 18000.0f;
}

static void MappingGrid_MarkFree(uint8_t x, uint8_t y)
{
  int8_t score;
  int8_t delta = -MAPPING_GRID_FREE_DELTA;

  if ((x >= MAPPING_GRID_WIDTH_CELLS) || (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return;
  }

  score = s_grid[y][x];
  if (score >= MAPPING_GRID_OCCUPIED_THRESHOLD)
  {
    delta = -MAPPING_GRID_FREE_OCCUPIED_DELTA;
  }
  else if (score > 0)
  {
    delta = -MAPPING_GRID_FREE_POSITIVE_DELTA;
  }

  MappingGrid_UpdateScore(x, y, delta);
  s_stats.free_ray_updates++;
}

static void MappingGrid_MarkOccupied(uint8_t x, uint8_t y, uint16_t distance_mm)
{
  int8_t delta = (distance_mm <= MAPPING_GRID_NEAR_WEAK_DISTANCE_MM) ?
      MAPPING_GRID_NEAR_OCCUPIED_DELTA :
      MAPPING_GRID_OCCUPIED_DELTA;

  MappingGrid_UpdateScore(x, y, delta);
  s_stats.occupied_updates++;
}

static void MappingGrid_ForceFree(uint8_t x, uint8_t y)
{
  int8_t delta;

  if ((x >= MAPPING_GRID_WIDTH_CELLS) || (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return;
  }

  delta = (int8_t)(MAPPING_GRID_FREE_THRESHOLD - s_grid[y][x]);
  if (delta > 0)
  {
    return;
  }

  MappingGrid_UpdateScore(x, y, delta);
}

static void MappingGrid_UpdateScore(uint8_t x, uint8_t y, int8_t delta)
{
  int16_t next_score;
  MappingGridCellState_t old_state;
  MappingGridCellState_t new_state;

  if ((x >= MAPPING_GRID_WIDTH_CELLS) || (y >= MAPPING_GRID_HEIGHT_CELLS))
  {
    return;
  }

  old_state = MappingGrid_GetCell(x, y);
  next_score = (int16_t)s_grid[y][x] + (int16_t)delta;
  if (next_score > MAPPING_GRID_MAX_SCORE)
  {
    next_score = MAPPING_GRID_MAX_SCORE;
  }
  else if (next_score < MAPPING_GRID_MIN_SCORE)
  {
    next_score = MAPPING_GRID_MIN_SCORE;
  }

  s_grid[y][x] = (int8_t)next_score;
  new_state = MappingGrid_GetCell(x, y);
  MappingGrid_ApplyStateChange(old_state, new_state);
}

static void MappingGrid_ApplyStateChange(MappingGridCellState_t old_state, MappingGridCellState_t new_state)
{
  if (old_state == new_state)
  {
    return;
  }

  switch (old_state)
  {
    case MAPPING_GRID_CELL_FREE:
      if (s_stats.free_cells > 0U) { s_stats.free_cells--; }
      break;
    case MAPPING_GRID_CELL_OCCUPIED:
      if (s_stats.occupied_cells > 0U) { s_stats.occupied_cells--; }
      break;
    case MAPPING_GRID_CELL_UNKNOWN:
    default:
      if (s_stats.unknown_cells > 0U) { s_stats.unknown_cells--; }
      break;
  }

  switch (new_state)
  {
    case MAPPING_GRID_CELL_FREE:
      s_stats.free_cells++;
      break;
    case MAPPING_GRID_CELL_OCCUPIED:
      s_stats.occupied_cells++;
      break;
    case MAPPING_GRID_CELL_UNKNOWN:
    default:
      s_stats.unknown_cells++;
      break;
  }
}

static void MappingGrid_TraceFreeRay(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y)
{
  int16_t x0 = (int16_t)start_x;
  int16_t y0 = (int16_t)start_y;
  int16_t x1 = (int16_t)end_x;
  int16_t y1 = (int16_t)end_y;
  int16_t dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
  int16_t sx = (x0 < x1) ? 1 : -1;
  int16_t dy = (y0 < y1) ? (y0 - y1) : (y1 - y0);
  int16_t sy = (y0 < y1) ? 1 : -1;
  int16_t error = dx + dy;

  for (;;)
  {
    int16_t e2;

    if ((x0 == x1) && (y0 == y1))
    {
      break;
    }

    MappingGrid_MarkFree((uint8_t)x0, (uint8_t)y0);
    e2 = (int16_t)(2 * error);

    if (e2 >= dy)
    {
      error = (int16_t)(error + dy);
      x0 = (int16_t)(x0 + sx);
    }

    if (e2 <= dx)
    {
      error = (int16_t)(error + dx);
      y0 = (int16_t)(y0 + sy);
    }
  }
}

static bool MappingGrid_FindClippedRayEnd(int32_t start_x_mm,
                                          int32_t start_y_mm,
                                          int32_t end_x_mm,
                                          int32_t end_y_mm,
                                          uint8_t *out_x,
                                          uint8_t *out_y)
{
  int32_t delta_x_mm = end_x_mm - start_x_mm;
  int32_t delta_y_mm = end_y_mm - start_y_mm;
  uint16_t steps;
  uint16_t step;
  uint8_t last_x = 0U;
  uint8_t last_y = 0U;
  bool found = false;

  if ((out_x == NULL) || (out_y == NULL))
  {
    return false;
  }

  steps = (uint16_t)((((delta_x_mm >= 0L) ? delta_x_mm : -delta_x_mm) +
                      ((delta_y_mm >= 0L) ? delta_y_mm : -delta_y_mm)) /
                     (int32_t)(MAPPING_GRID_CELL_SIZE_MM / 2U));
  if (steps == 0U)
  {
    steps = 1U;
  }

  for (step = 0U; step <= steps; ++step)
  {
    int32_t x_mm = start_x_mm + ((delta_x_mm * (int32_t)step) / (int32_t)steps);
    int32_t y_mm = start_y_mm + ((delta_y_mm * (int32_t)step) / (int32_t)steps);
    uint8_t cell_x;
    uint8_t cell_y;

    if (MappingGrid_WorldToCell(x_mm, y_mm, &cell_x, &cell_y))
    {
      last_x = cell_x;
      last_y = cell_y;
      found = true;
    }
    else if (found)
    {
      break;
    }
  }

  if (!found)
  {
    return false;
  }

  *out_x = last_x;
  *out_y = last_y;
  return true;
}

static char MappingGrid_CellToChar(MappingGridCellState_t state)
{
  switch (state)
  {
    case MAPPING_GRID_CELL_FREE:
      return '.';
    case MAPPING_GRID_CELL_OCCUPIED:
      return '#';
    case MAPPING_GRID_CELL_UNKNOWN:
    default:
      return '?';
  }
}
