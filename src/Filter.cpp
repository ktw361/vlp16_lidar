#include "vlp16_lidar/Filter.h"
#include "vlp16_lidar/velodyne.h"
#include <iostream>

Filter::Filter(int rows, int cols, int order_set)
{
	row_num = rows;
	col_num = cols;
	order = order_set;
	z0_tmp = int_2d_init(row_num, col_num);
	for (int i=1;i!=order;i++) {
		z.push_back(int_2d_init(row_num,col_num));
	}

}

// Note: Input should be bool
int **
Filter::moving_average_with_update(int** z0) 
{
	for (int i=0;i!=row_num;i++) {
		for (int j=0;j!= col_num;j++) {
			z0_tmp[i][j] = z0[i][j]; 			// z0[i][j] = z0_tmp[i][j];
			for (auto &c:z) 
				z0[i][j] += c[i][j];
			z0[i][j] = (z0[i][j] > (order / 2));
			// update
			for (int k=order-2;k>0;k--) 
				z[k][i][j] = z[k-1][i][j];
			z[0][i][j] = z0_tmp[i][j];
		}
	}
	return z0;
}

Filter::~Filter()
{
	int_2d_free(z0_tmp,row_num);
	z.clear();
    std::cout << "Filter Destruct Success" << '\n';
}
