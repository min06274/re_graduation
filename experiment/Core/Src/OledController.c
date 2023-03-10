#include "OledController.h"
#include "eddy_logo.h"
#include <stdio.h>


void opening(double temp)
{

	double answer = temp/15;

	answer *=1000;

	uint32_t aaa = answer;






	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);


	SSD1306_DrawBitmap(0, 52, logo0, 128, 12, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(aaa);

	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo1, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);

	SSD1306_DrawBitmap(0, 52, logo2, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);

	SSD1306_DrawBitmap(0, 52, logo3, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo4, 128, 64, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo5, 128, 64, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo6, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo7, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo8, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo9, 128, 64, 1);
	SSD1306_UpdateScreen();


	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo10, 128, 64, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo11, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo12, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo13, 128, 64, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt ing...", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo14, 128, 64, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(aaa);
	SSD1306_Clear();

	SSD1306_GotoXY(0, 0);

	SSD1306_Puts("salt end", &Font_11x18, 1);
	SSD1306_DrawBitmap(0, 52, logo15, 128, 64, 1);
	SSD1306_UpdateScreen();

	//printDefault();
}


void printDefault(){

	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("Servo Angle", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("---------", &Font_11x18, 1);
	SSD1306_GotoXY(14, 38);
	SSD1306_Puts("0 degree", &Font_11x18, 1);



	SSD1306_UpdateScreen();


}

void printTemper(int temper)
{
	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str,"%d",temper);

	SSD1306_Puts(temper_str, &Font_11x18, 1);


	SSD1306_UpdateScreen();


}
