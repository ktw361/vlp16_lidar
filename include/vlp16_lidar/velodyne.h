#ifndef VELODYNE_H
#define VELODYNE_H

#include "MeshGrid.h"

#define char_to_int(chr) ((chr) & 0xff)

#define PI 					3.14159265

#define Z_RESOLUTION		1						// z axis resolution 1 cm
#define Z_GROUND			(-150/Z_RESOLUTION)		// ground height relative to lidar , 150cm
#define Z_CEILING			(40/Z_RESOLUTION)		// lowest height that we care about, 20cm above lidar
#define GROUND_THRESH 		(5/Z_RESOLUTION)		// ground if within [Z_GROUND - GROUND_THRESHOLD, Z_GROUND + GROUND_THRESHOLD]

#define SEARCH_IDX 			(40/10)		
#define WINDOW_SZ 			 SEARCH_IDX

extern double SIN_W[16];
extern double COS_W[16];

int get_abs_azimuth(char *);
int get_abs_dist(char *, int);

int** int_2d_init(int row_num, int col_num);
void int_2d_free(int** array, int row_num);
void int_2d_clear(int** array, int row_num, int col_num);

#endif