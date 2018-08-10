#include <vector>
#include "vlp16_lidar/velodyne.h"
#include "vlp16_lidar/MeshGrid.h"
#include "vlp16_lidar/T_FU_DATA.h"

double SIN_W[16] = {-0.25881904510252074,
 0.017452406437283512,
 -0.22495105434386498,
 -0.052335956242943828,
 -0.1908089953765448,
 0.087155742747658166,
 -0.15643446504023087,
 0.12186934340514748,
 -0.12186934340514748,
 0.15643446504023087,
 -0.087155742747658166,
 0.1908089953765448,
 -0.052335956242943828,
 0.22495105434386498,
 -0.017452406437283512,
 0.25881904510252074};

double COS_W[16] = {0.96592582628906831,
 0.99984769515639127,
 0.97437006478523525,
 0.99862953475457383,
 0.98162718344766398,
 0.99619469809174555,
 0.98768834059513777,
 0.99254615164132198,
 0.99254615164132198,
 0.98768834059513777,
 0.99619469809174555,
 0.98162718344766398,
 0.99862953475457383,
 0.97437006478523525,
 0.99984769515639127,
 0.96592582628906831};

int 
get_abs_azimuth(char * block)
{
    return ((char_to_int(block[1]) << 8) | char_to_int(block[0]));
}

int 
get_abs_dist(char * block, int  i)
{
    return ((char_to_int(block[i+1]) << 8 )| char_to_int(block[0]));
}

int **
int_2d_init(int row_num, int col_num)
{
	int ** array = new int*[row_num];
	for (int i=0;i!=row_num;i++)
		array[i] = new int[col_num]();
	return array;
}

void 
int_2d_free(int** array, int row_num)
{
	if (!array)	return;
	for (int i=0;i!=row_num;i++)
		if (array[i])
			delete [] array[i];
	delete [] array;
}

void
int_2d_clear(int** array, int row_num, int col_num)
{
	if (!array) return;
	for (int i=0;i!=row_num;i++)
		for (int j=0;j!=col_num;j++)
			array[i][j] = 0;
}