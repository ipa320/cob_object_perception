#include "get_mapping.h"
//#include "categorization_fkt.h"
#include <cmath>
#include <string>
//#include <lbp.h>
//#include <svm.h>


int count_1bit(int n)
{
//counts number of 1-Bits
	int count = 0;
	for(int i = sizeof(n)*8-1; i >= 0; i-- )
	{
		if((n & 1) == 1)
		{
			count++;
		}
		n = n >> 1;
	}


   return count ;
}

int transition_number(int val1, int val2)
{
//counts number of 1->0 and 0->1 transitions
	int res = 0;
	if((val1 & (1 << 7)) > 0)
	{
		val2 |= (1<<0);
		val2 &= ~(1<<8);
	}
	res = count_1bit(val1^val2);
	return res;
}

get_mapping::get_mapping()
{

}

void get_mapping::get_mapping_res(int samples, std::string mappingtype, struct mapping *map, int *table)
{


for(int i = 0; i<=pow(2,samples); i++) table[i] = -1;
int newMax = 0;
int index = 0;

if(mappingtype == "u2")
{
	newMax = samples*(samples-1) + 3;
	for(int i = 0; i<=pow(2,samples) - 1; i++)
	{
		int j = i << 1;
		int numt = transition_number(i,j);

		if(numt <= 2)
		{
			(table)[i+1] = index;
			index = index + 1;
		}
		else
		{
			table[i+1] = newMax - 1;
		}
	}
}

if(mappingtype == "ri")
{
	int index = static_cast<int>(pow(2,samples));
	double tmpMap[index];
	for(int i = 0; i<=pow(2,samples)-1; i++)
	{
		tmpMap[i] = -1;
		int rm = i;
		int r = i;
		for(int j = 1; j<=samples - 1; j++)
		{
			r = r << 1;
			if(r < rm)
			{
				rm = r;
			}
		}
		if(tmpMap[rm + 1] < 0)
		{
			tmpMap[rm + 1] = newMax;
			newMax = newMax + 1;
		}
		table[i + 1] = tmpMap[rm + 1];
	}
}

if(mappingtype == "riu2")
{
	newMax = samples + 2;
	for(int i = 0; i<=pow(2,samples) - 1; i++)
	{
		int j = i << 1;
		int numt = transition_number(i,j);
		if(numt <= 2)
		{
			table[i] = count_1bit(i);
		}
		else
		{
			table[i] = samples + 1;
		}
	}
}

//for(int i = 0; i<256; i++)
//{
//	std::cout << table[i] << "  ";
//}


(*map).samples=samples;
(*map).num=newMax;
}
