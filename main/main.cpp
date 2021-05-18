
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "font.h"
#include "OLED.h"


extern "C" void app_main();

void app_main(void)
{
	OLED oled( 128, 64, 4, 15, 16 );
	oled.setFont(ArialMT_Plain_16 );
	oled.clear();
	oled.sendData();

    vTaskDelay( 500 / portTICK_PERIOD_MS);

    int col = 0;

    OLEDDISPLAY_COLOR color = WHITE;

    for ( ;; )
    {
    	char buf[100];

    	oled.clear();
    	sprintf( buf, "Cnt: %06d", col );
    	oled.drawString( 0, 0, buf, color );
    	oled.sendDataBack();

/*
    	for ( int i = 0 ; i < 64 ; i++ )
        	setPixelColor( i+col, i, color );
    	sendData();
*/
        col++;

        //vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}


