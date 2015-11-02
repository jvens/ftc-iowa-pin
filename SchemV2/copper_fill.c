#include "stdio.h"
#include "stdlib.h"


int main(int argc, char** argv)
{
	FILE* fp = fopen(argv[1], "r");
	char* line = NULL;
	char* ptr;
	size_t size = 0;
	
	while(getline(&line, &size, fp) != -1)
	{
		double x, y;
		sscanf(line, " %*s %*s %lf %lf ", &x, &y);
		printf("(xy %f %f) ", x, y);
	}
	
	return 0;
}
