#ifndef FILTER_H
#define FILTER_H

#include <vector>

class Filter{
public:
	Filter(int,int,int order= 3);
	~Filter();
	int** moving_average_with_update(int**);// Note: Input should be bool

private:
	int row_num, col_num;
	int order;
	int **z0_tmp;
	std::vector<int**> z;
};

#endif