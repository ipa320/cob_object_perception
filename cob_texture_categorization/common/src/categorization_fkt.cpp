#include "categorization_fkt.h"
#include <stdio.h>
#include <stdlib.h>
//#include <lbp.h>



int categorization_fkt::count_1bit(int n)
{
//counts number of 1-Bits
	int count = 0;

	for(double i = sizeof(n)*8-1; i >= 0; i-- )
	{
		if((n & 1) == 1)
		{
			count++;
		}
		n = n >> 1;
	}


   return count ;
}

int categorization_fkt::transition_number(int val1, int val2)
{
//counts number of 1->0 and 0->1 transitions
	int res = 0;
	res = count_1bit(val1 ^ val2);
	return res;
}





