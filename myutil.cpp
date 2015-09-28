/*
 * myutil.cpp
 *
 * 
 */ 

void convertToDecimalString(char * buf, int n)
{

	char * str = buf;
	char c;
	int m;
	
	buf[1] = 0;
	buf[2] = 0;
	
	if(n > 99)
	{
		// 3-digit no.
		str += 2;
	}
	else if(n > 9)
	{
		// 2-digit no.
		str += 1;
	}

	do 
	{
		m = n;
		n /= 10;
		c = m - 10 * n;
		*str = (c < 10) ? (c + '0') : (c + 'A' - 10);
		--str;
	} 
	while(n);
	
}

int sonarFilter(int Arr[])
{
	int size = sizeof(Arr);
	int sum = 0;
	int i;
	int count = size;
	for(i=0; i<size; i++){
		sum += Arr[i];
	}
	for(i=0; i<szie; i++){
		int diff = Arr[i] - (sum/size);
		if(diff > 200 || diff < -200){
			sum-=Arr[i];
			count--;
		}
	}
	return sum/count;
}