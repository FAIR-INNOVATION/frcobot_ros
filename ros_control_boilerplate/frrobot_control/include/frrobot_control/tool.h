#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define BYTE unsigned char
#define ENCRESOLUTION 16384
#define GEARRATIO 101

int BytesToString(BYTE* pSrc, char* pDst, int nSrclength);
int StringToBytes(char* pSrc, BYTE* pDst, int nSrclength);

double EncToRadian(int32_t encValue);
int32_t RadianToEnc(double angle);