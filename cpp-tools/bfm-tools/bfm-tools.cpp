#include "bfm.h"

int main()
{
	if (!init_bfm())
		return -1;
	generate_random_face();
	system("pause");
    return 0;
}
