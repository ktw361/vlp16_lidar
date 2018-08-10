#ifndef MESHGRID_H
#define MESHGRID_H

#include <vector>
#include <deque>
#include <list>
#include <algorithm>
#include "T_FU_DATA.h"
#include "velodyne.h"
#include "Filter.h"

	// Layout :
	//  ^
	//  |
	//  | i(V)(Moving Direction,Y)
	//  |
	//  |
	//  ------------>
	//      j (H)
class MeshGrid
{
	typedef std::pair<int,int> MeshPoint;
	typedef std::deque<MeshPoint> PointList;
	enum {
		HOLE = -2,
		GROUND = 0,
		OBSTACLE = 1,	
		FLOATING = 0, 	// something that lowest part is floating above lidar , i.e z > Z_CEILING
	};

public:
	// [][] == [h][v]
	// num_elem[][] store number of elements in one grid
	// low[][], high[][], diff[][] store the height data in one grid
	// grid map is stored Horizontally first
	int **num_elem;
    int **low, **high;
    int **diff;
    int **cell_content; // defines the content type in one cell: [GROUND, OBSTACLE, FLOATING]
    int **examined;
    int ** grid_to_obs_idx;

    std::list<MeshPoint>obs_points;
    std::vector<PointList> obstacle_list; // each element in vector is a collection of all points for one obstacle

	MeshGrid();
	~MeshGrid();
	std::vector<PointList> merge_obstacles();
	int get_type(int, int, int);
	int consume_udp(char *);
	void grid_set(int, int, int);
	void visualize_text();
	void clear();

	MeshPoint min_row_of_col(int**,int, int);
	T_3D16_OBS_DATA compute_obs_data(int);
	T_3D16_OBS_TO_FU get_obs_to_fu();
	T_3D16_GRID_TO_FU get_grid_to_fu();
private:
	Filter mesh_filter;
	int past_angle;
	char buf[2];
	char block_cur[98];
	char block_next[98];
};

template <typename T> T ros_int_init(int data);

#endif
