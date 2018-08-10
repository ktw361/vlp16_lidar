#ifndef FILTER_H
#define FILTER_H

class Filter{
public:
	Filter(int,int);
	~Filter();
	int** moving_average_with_update(int**);// Note: Input should be bool

private:
	int row_num, col_num;
	int **z0_tmp;
	int **z1;
	int **z2;
};

#endif