#include "frrobot_control/tool.h"

int BytesToString(BYTE* pSrc, char* pDst, int nSrclength)
{
	const char tab[] = "0123456789ABCDEF";    //0x0-0xF 的字符查找表

	//转换字符串
	int i = 0;
	for(i = 0;i < nSrclength;i++)
	{
		*pDst++ = tab[*pSrc >> 4];		//输出高四位；
		*pDst++ = tab[*pSrc & 0x0f];	//输出低四位;
		*pDst++ = ' ';
		pSrc++;
	}

	//输出字符串加结束符
	*pDst = '\0';

	//返回目标字符串长度
	return (nSrclength * 3);
}



int StringToBytes(char* pSrc, BYTE* pDst, int nSrclength)
{
	int i = 0;

	for(i = 0;i < nSrclength;i++)
	{
		//输出高四位
		if ((*pSrc >= '0') && (*pSrc <= '9'))
		{
			*pDst = (*pSrc - '0') << 4;
		}
		else if ((*pSrc >= 'A') && (*pSrc <= 'Z'))
		{
			*pDst = (*pSrc - 'A' + 10) << 4; 
		}
		else
		{
			*pDst = (*pSrc - 'a' + 10) << 4; 
		}

		pSrc++;

		//输出低四位
		if ((*pSrc >= '0') && (*pSrc <= '9'))
		{
			*pDst |= *pSrc - '0';
		}
		else if ((*pSrc >= 'A') && (*pSrc <= 'Z'))
		{
			*pDst |= *pSrc - 'A' + 10; 
		}
		else
		{
			*pDst |= *pSrc - 'a' + 10; 
		}

		pSrc++;
		pDst++;

		pSrc++;
	}
	//返回目标数据长度
	return nSrclength;
}

double EncToRadian(int32_t encValue) {
	return ((double)encValue/ENCRESOLUTION*360.0/GEARRATIO)*M_PI/180.0;
}

int32_t RadianToEnc(double radian) {
	return (radian*180.0/M_PI)*GEARRATIO/360.0*ENCRESOLUTION;
}