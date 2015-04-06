
static int linearInterp(int x_key, int x1, int x2, int y1, int y2)
{
	return y1 + (long(y2 - y1)*(x_key - x1)) / (x2 - x1);
}

static char findIndexJustAbove(const int array[], int key, int length)
{		// assumes the input array is sorted high to low. 
	int i;
	for (i = 0; i < length; i++)
	{
		if (array[i] < key)
			return i;
	}
	// didn't find a value above key.  Return -1 to indicate an error. 
	return -1;
}