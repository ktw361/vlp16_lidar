#include "vlp16_lidar/Filter.h"
#include "vlp16_lidar/velodyne.h"
#include <iostream>

Filter::Filter(int rows, int cols)
{
	row_num = rows;
	col_num = cols;
	z0_tmp = int_2d_init(row_num, col_num);
	z1 = int_2d_init(row_num, col_num);
	z2 = int_2d_init(row_num, col_num);
}

// Note: Input should be bool
int **
Filter::moving_average_with_update(int** z0) 
{
	for (int i=0;i!=row_num;i++) {
		for (int j=0;j!= col_num;j++) {
			z0_tmp[i][j] = z0[i][j];
			z0[i][j] = (z0_tmp[i][j] + z1[i][j] + z2[i][j]) > 1; // if 2 out of 3 is one, then 1; 0 otherwise
			// update
			z2[i][j] = z1[i][j];
			z1[i][j] = z0_tmp[i][j];
		}
	}
	return z0;
}

Filter::~Filter()
{
	int_2d_free(z0_tmp,row_num);
	int_2d_free(z1,row_num);
	int_2d_free(z2,row_num);
    std::cout << "Filter Destruct Success" << '\n';
}