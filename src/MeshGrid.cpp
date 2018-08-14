#include <iostream>
#include <vector>
#include <list>
#include <cstring>
#include "vlp16_lidar/T_FU_DATA.h"
#include "vlp16_lidar/velodyne.h"
#include "vlp16_lidar/MeshGrid.h"
#include "vlp16_lidar/Filter.h"

#define PAST_ANGLE (36000 * 2)
#define FILTER_ORDER 5
using std::cout;
using std::endl;

MeshGrid::MeshGrid() : mesh_filter(T_GRID_H_NUM,T_GRID_V_NUM,FILTER_ORDER)
{
	num_elem = int_2d_init(T_GRID_H_NUM, T_GRID_V_NUM);
	low = int_2d_init(T_GRID_H_NUM, T_GRID_V_NUM);
	high = int_2d_init(T_GRID_H_NUM, T_GRID_V_NUM);
	diff = int_2d_init(T_GRID_H_NUM, T_GRID_V_NUM);
	cell_content = int_2d_init(T_GRID_H_NUM, T_GRID_V_NUM);
	examined = int_2d_init(T_GRID_H_NUM,T_GRID_V_NUM);
	grid_to_obs_idx = int_2d_init(T_GRID_H_NUM,T_GRID_V_NUM);
}

// This function takes as input only cell_content and obs_point, which is a aux vector
std::vector<MeshGrid::PointList> 
MeshGrid::merge_obstacles()
{
	// Step one: do filtering
	mesh_filter.moving_average_with_update(cell_content);

	// Step two: reset obs_point based on  cell_content
    obs_points.clear();
	for (int i=0;i!=T_GRID_H_NUM;i++)
		for (int j=0;j!=T_GRID_V_NUM;j++)
			if (cell_content[i][j] == OBSTACLE) 
				obs_points.push_back(MeshPoint({i,j}));

	// Begin merge
	MeshPoint cur_point;
	PointList search_obstacle;
	std::list<MeshPoint>::iterator it;
	int_2d_clear(examined,T_GRID_H_NUM,T_GRID_V_NUM);

	while(!obs_points.empty()){
		PointList cur_obstacle;
		cur_point = obs_points.front();
		obs_points.pop_front();
		search_obstacle.push_back(cur_point);
		
		while(!search_obstacle.empty()){
			cur_point = search_obstacle.front();
			search_obstacle.pop_front();
			cur_obstacle.push_back(cur_point);

    		for (int i=std::max(cur_point.first - SEARCH_IDX,0);i<=std::min(cur_point.first + SEARCH_IDX,T_GRID_H_NUM-1);i++){
    			for (int j=std::max(cur_point.second - SEARCH_IDX,0);j<=std::min(cur_point.second + SEARCH_IDX,T_GRID_V_NUM-1);j++){
    				if (cell_content[i][j] != OBSTACLE) continue; // if not a obstacle, continue
    				if ((i==cur_point.first && j==cur_point.second)) continue; // if detecting current location, continue
    				if (examined[i][j]) continue; // if this location has been examined, continue
    				
    				// now merge obs
					it = std::find(obs_points.begin(), obs_points.end(), std::make_pair(i,j));
					if (it != obs_points.end()) {
    					search_obstacle.push_back(*it);
    					obs_points.erase(it);
					}
    				examined[i][j] = 1;
    			}
    		}
    	}
    	obstacle_list.push_back(cur_obstacle);
	}

	for (int i=0;i!=obstacle_list.size();i++){ // computer grid_to_obs_idx
		for (auto &c: obstacle_list[i]){
			grid_to_obs_idx[c.first][c.second] = i + 1;
		}
	}

	return obstacle_list;
}

int 
MeshGrid::get_type(int z_low, int z_high, int diff) // returns content type according to z and diff
{
	if (diff > GROUND_THRESH)
		return OBSTACLE;
	else
		return GROUND;
}

void 
MeshGrid::grid_set(int x,int y, int new_z) // generate a point in GridMap from xyz
{
	// unit in udp is 2mm, unit in Ros is 1cm,
	// so, index_in_GridMap = distance * 2e-3 / (RESOLUTION * 1e-2)
	// that is inex_in_GM = distance  / (5 * RESOLUTION)
	// Also conver z to meter.
    x = x / (5 * T_GRID_RESOLUTION_H_CM) + (T_GRID_H_NUM / 2);
    y = y / (5 * T_GRID_RESOLUTION_V_CM);
    new_z = new_z / (5 * Z_RESOLUTION);

    if (x < 0 || x >= T_GRID_H_NUM) return;
    if (y < 0 || y >= T_GRID_V_NUM)	return;
    if ((x<16 && x>4) && (y<6) && (new_z < 30 && new_z > -30)) return; // Clear noise around center
    if (new_z >= Z_CEILING) return; // Clear top
    if(num_elem[x][y] == 0){
        low[x][y] = new_z;
        high[x][y] = new_z;
    }
    else{
        low[x][y] = std::min(low[x][y],new_z);
        high[x][y] = std::max(high[x][y], new_z);
        diff[x][y] = high[x][y] - low[x][y];
    }

    int content_old = cell_content[x][y];
    cell_content[x][y] = get_type(low[x][y], high[x][y], diff[x][y]); // another way: if num_elem[x][y] >= 2, then OBSTACLE

//    if (cell_content[x][y] == OBSTACLE && content_old != OBSTACLE)
//    	obs_points.push_back(MeshPoint({x,y}));

	num_elem[x][y] ++;
}

MeshGrid::MeshPoint 
MeshGrid::min_row_of_col(int ** grid_to_obs_idx, int col, int obs_idx) // Will find at least one 
{
	MeshPoint min_point = std::make_pair(col,T_GRID_V_NUM);
	for (int v=T_GRID_V_NUM-1;v>=0;v++){ // serach backwards
		int obs_idx_temp = grid_to_obs_idx[col][v];
		if (obs_idx_temp != obs_idx) continue;
		min_point = std::make_pair(col,v);
	}
	return min_point;
}

void
MeshGrid::compute_obs_data(vlp16_lidar::T_Msg_3D16_OBS_TO_FU_<std::allocator<void> >::_pObs_type &pObs, int obs_id)
{
	// Real computation
	PointList p_list = obstacle_list[obs_id];
	int x_min = p_list[0].first, x_max = p_list[0].first;
	int y_min, y_max = p_list[0].second;
	int x_min_y, x_max_y, y_min_x, y_max_x;
	for (auto &c: p_list){
		if (c.first < x_min)
			x_min = c.first, x_min_y = c.second;
		if (c.first > x_max)
			x_max = c.first, x_max_y = c.second;
		if (c.second > y_max)
			y_max = c.second, y_max_x = c.first;
	}
	int middle_idx = (x_min + x_max) / 2;
	x_min = T_GRID_V_NUM + 1;
	for (int i=std::max(middle_idx - WINDOW_SZ/2,x_min); i<=std::min(middle_idx + WINDOW_SZ/2,x_max);i++){
		MeshPoint min_point_temp = min_row_of_col(grid_to_obs_idx, i , obs_id);
		if (min_point_temp.second < x_min)
			x_min = min_point_temp.first, x_min_y = min_point_temp.second;
	}

	// Setting ptr
	auto pObs_ptr = pObs.begin() + 10*obs_id;
	pObs_ptr[0] = obs_id;
	pObs_ptr[1] =  x_min*T_GRID_RESOLUTION_V_CM, pObs_ptr[2] =  x_min_y*T_GRID_RESOLUTION_H_CM;
	pObs_ptr[3] =  x_max*T_GRID_RESOLUTION_V_CM, pObs_ptr[4] =  x_max_y*T_GRID_RESOLUTION_H_CM;
	pObs_ptr[5] =  y_min_x*T_GRID_RESOLUTION_V_CM, pObs_ptr[6] =  y_min*T_GRID_RESOLUTION_H_CM;
	pObs_ptr[7] =  y_max_x*T_GRID_RESOLUTION_V_CM, pObs_ptr[8] =  y_max*T_GRID_RESOLUTION_H_CM;
	pObs_ptr[9] = p_list.size();
}

void 
MeshGrid::set_obs_msg(vlp16_lidar::T_Msg_3D16_OBS_TO_FU & obs_msg)
{
	obs_msg.nObs = obstacle_list.size();
	for (int i=0;i!=obstacle_list.size();i++)
		compute_obs_data(obs_msg.pObs,i);
}

void 
MeshGrid::set_grid_msg(vlp16_lidar::T_Msg_3D16_GRID_TO_FU & grid_msg)
{
	grid_msg.nObs = obstacle_list.size();
	for (int v=0;v!=T_GRID_V_NUM;v++) {
		for (int h=0;h!=T_GRID_H_NUM;h++) {
			grid_msg.gridMsk[v*T_GRID_H_NUM + h] = cell_content[h][v];
		}
	}
	for (int i=0;i!=obstacle_list.size();i++) {
		int j=0;
		for (auto &c: obstacle_list[i])
			grid_msg.pObs[i*T_3D16_OBS_MAX_GRID_NUM + j++] = c.first * T_GRID_H_NUM + c.second;
		// Now j should be equal to obstacle_list[i].size()
		grid_msg.pObs[i*T_3D16_OBS_MAX_GRID_NUM + obstacle_list[i].size()] = -1;
	}
}

void
MeshGrid::visualize_text() // Call this function after merge_obstacles
{
	for (int v=0;v!=T_GRID_V_NUM;v++)
		cout << '-';
	cout << endl;
	for (int h=0;h!=T_GRID_H_NUM;h++){
		for (int v=0;v!=T_GRID_V_NUM;v++){
			if (grid_to_obs_idx[h][v])
				cout << grid_to_obs_idx[h][v];
			else
				cout << ' ';
		}
		cout << endl;
		if (h == T_GRID_H_NUM/2 - 1){
			cout << '>';
			for (int v=1;v!=T_GRID_V_NUM;v++)
				cout << '-';
			cout << endl;
		}
	}
	for (int v=0;v!=T_GRID_V_NUM;v++)
		cout << '-';
	cout << endl;
	cout << "             ----------(Moving Direction)Y----------->     \n\n";
}

int 
MeshGrid::consume_udp(char * udp_packet)
{
	char * udp_buf = udp_packet;

	while(strncpy(buf,udp_buf,2)){
		udp_buf += 2;
		if(buf[0] == '\xff' && buf[1] == '\xee') {
			strncpy(block_cur, udp_buf,98);
			udp_buf += 98;
			break;
		}
	}
	for (int i=0;i!=11;i++) {
		// Consume next block to get azimuth interpolation
		strncpy(buf, udp_buf,2);
		udp_buf += 2;
		if (buf[0] != '\xff' && buf[1] != '\xee') {
			cout << "Consume UDP Error" << endl;
			return 0;
		}
		strncpy(block_next, udp_buf, 98);
		udp_buf += 98;

		// Interpolate azimuth
        int azimuth_cur = get_abs_azimuth(block_cur);
        int azimuth_next = get_abs_azimuth(block_next);
        int circled_flag = (azimuth_next < azimuth_cur);
        azimuth_next = ((!circled_flag) ? (azimuth_next) : (azimuth_next + 36000));
        int azimuth_mid = (azimuth_next + azimuth_cur ) / 2;
        
        //First 16 channel
        double sin_alpha = sin(azimuth_cur * PI / 18000);
        double cos_alpha = cos(azimuth_cur * PI / 18000);  
        for(int channel_id =0;channel_id!=16;channel_id++){
            int abs_dist = get_abs_dist(block_cur, 3*channel_id + 2);
            int x = abs_dist * COS_W[channel_id] * sin_alpha;
            int y = abs_dist * COS_W[channel_id] * cos_alpha;
            int z = abs_dist * SIN_W[channel_id];
            grid_set(x,y,z);
        }

        // Second 16 channels
        sin_alpha = sin(azimuth_mid * PI / 18000);
        cos_alpha = cos(azimuth_mid * PI / 18000);
        int OFFSET = 48;
        for(int channel_id=0;channel_id!=16;channel_id++){
            int abs_dist = get_abs_dist(block_cur, 3*channel_id + 2 + OFFSET);
            int x = abs_dist * COS_W[channel_id] * sin_alpha;
            int y = abs_dist * COS_W[channel_id] * cos_alpha;
            int z = abs_dist * SIN_W[channel_id];
            grid_set(x,y,z);
        }

        // If finish one circle, terminate
        past_angle += (azimuth_next - azimuth_cur);
        strcpy(block_cur, block_next);
        if (past_angle > PAST_ANGLE) {
			past_angle = 0;
        	merge_obstacles();
			// visualize_text();
        	// clear();
        	return 1;
        }
	}
	return 0;
}


void
MeshGrid::clear()
{
	int_2d_clear(num_elem, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(low, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(high, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(diff, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(cell_content, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(examined, T_GRID_H_NUM, T_GRID_V_NUM);
	int_2d_clear(grid_to_obs_idx,T_GRID_H_NUM,T_GRID_V_NUM);
	obs_points.clear();
	obstacle_list.clear();
}

MeshGrid::~MeshGrid()	// destruct
{
	int_2d_free(low,T_GRID_H_NUM);
	int_2d_free(high,T_GRID_H_NUM);
	int_2d_free(diff,T_GRID_H_NUM);
	int_2d_free(num_elem,T_GRID_H_NUM);
	int_2d_free(examined,T_GRID_H_NUM);
	int_2d_free(grid_to_obs_idx,T_GRID_H_NUM);
	obs_points.clear();
	obstacle_list.clear();
    cout << "MeshGrid Destruct Success" << '\n';
}

template <typename T>
T
ros_int_init(int data)
{
	T ret;
	ret.data = data;
	return ret;
}
