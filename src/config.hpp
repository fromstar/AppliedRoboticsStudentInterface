#ifndef __CONNECTOR_HPP__
#define __CONNECTOR_HPP__

#define RES 3
#define AGGREGATE 0.0
#define OFFSET 105e-3
#define DIAGONAL_MOVE_COST 3
#define PERPENDICULAR_MOVE_COST 2
#define FUGITIVE_MOVE_COST_FOR_CATCHER 0  // When catcher moves this is the
                                          // base cost of its action.
#define KMAX 40

#define DIM_X_PLOT 600
#define DIM_Y_PLOT 900
#define PLOT_CELL_ID false

#define N_MAX_CURVES 500
#define SEARCH_ANGLE M_PI
#define REFINE 0.05
#define RESOLUTION_STEP 20 * (M_PI / 180); // n degree converted in radiants

#endif
