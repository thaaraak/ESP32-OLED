#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "font.h"

#define OLED_ADDRESS 0x3C

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define RESET_PIN 			16

#define I2C_MASTER_SCL_IO 	15               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 	4               /*!< gpio number for I2C master data  */

#define I2C_MASTER_NUM 		1
#define I2C_MASTER_FREQ_HZ 	100000						      /*!< I2C master clock frequency */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3


// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2


int writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
uint16_t readRegister(uint8_t reg);

uint8_t *_buffer;
uint8_t *_buffer_back;

int _displayWidth;
int _displayHeight;
int _displayBufferSize;


int writeRegister( uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buffer[3];

    //printf( "Writing [%d]=[%d] ", reg, value );

    buffer[0] = reg;
    buffer[1] = value;

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, &buffer[0], 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK( (ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS) ) );
    i2c_cmd_link_delete(cmd);

    //printf( " Return: %d\n", ret );


    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

int writeData( uint8_t addr, uint8_t* buf, int len )
{
    uint8_t buffer = 0x40;

    //printf( "Writing Data: [%d]\n", len );

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, buffer, ACK_CHECK_EN);

    i2c_master_write(cmd, buf, len - 1, ACK_VAL);
    i2c_master_write_byte(cmd, buf[len - 1], NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //printf( " Return: %d\n", ret );


    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 800000;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void sendCommand( uint8_t command )
{
	writeRegister( OLED_ADDRESS, 0x80, command );
}

void sendData()
{
	sendCommand(COLUMNADDR);
	sendCommand(0);
	sendCommand((_displayWidth - 1));

	sendCommand(PAGEADDR);
	sendCommand(0x0);
	sendCommand(0x7);

	for ( uint16_t i = 0; i < _displayBufferSize/16; i++)
	{
		writeData( OLED_ADDRESS, &_buffer[i*16], 16 );
	}

}

void sendDataBack()
{

	uint8_t minBoundY = UINT8_MAX;
	uint8_t maxBoundY = 0;

	uint8_t minBoundX = UINT8_MAX;
	uint8_t maxBoundX = 0;
	uint8_t x, y;


	// In double buffered mode we first determine the area of difference
	// between the current write buffer and the previously saved back buffer
	// These bounds become the start column and page later on

	for (y = 0; y < _displayHeight / 8 ; y++)
	{
		for (x = 0; x < _displayWidth; x++)
		{
			uint16_t pos = x + y * _displayWidth;
			if (_buffer[pos] != _buffer_back[pos])
			{
				minBoundY = fmin(minBoundY, y);
				maxBoundY = fmax(maxBoundY, y);
				minBoundX = fmin(minBoundX, x);
				maxBoundX = fmax(maxBoundX, x);
			}
			_buffer_back[pos] = _buffer[pos];
		}
		//yield();
	}

	if (minBoundY == UINT8_MAX) return;

	// Set the start column and page to the previously calulated bounds

	sendCommand(COLUMNADDR);
	sendCommand( minBoundX);
	sendCommand( maxBoundX);

	sendCommand(PAGEADDR);
	sendCommand(minBoundY);
	sendCommand(maxBoundY);

	// Now write out the changed portion of the buffer in 16 byte blocks

	uint8_t sbuf[16];
	int idx = 0;

	for (y = minBoundY; y <= maxBoundY; y++)
	{
		for (x = minBoundX; x <= maxBoundX; x++)
		{
			sbuf[idx++] = _buffer[x + y * _displayWidth];
			if ( idx == 16 )
			{
				writeData( OLED_ADDRESS, &sbuf[0], 16 );
				idx = 0;
			}
		}
	}

	// Finally write out the remaining left over block

	if ( idx > 0 )
		writeData( OLED_ADDRESS, &sbuf[0], idx );


}



void initializeReset( int reset )
{
	gpio_num_t r = (gpio_num_t) reset;

    gpio_pad_select_gpio( r );
    gpio_set_direction( r, GPIO_MODE_OUTPUT);

    gpio_set_level(r, 0);
    vTaskDelay( 50 / portTICK_PERIOD_MS);
    gpio_set_level(r, 1);
    vTaskDelay( 50 / portTICK_PERIOD_MS);

}

void initializeOLED(void)
{
	sendCommand(DISPLAYOFF);
	sendCommand(SETDISPLAYCLOCKDIV);
	sendCommand(0xF0); // Increase speed of the display max ~96Hz
	sendCommand(SETMULTIPLEX);
	sendCommand(_displayHeight - 1);
	sendCommand(SETDISPLAYOFFSET);
	sendCommand(0x00);
	sendCommand(SETSTARTLINE);
	sendCommand(CHARGEPUMP);
	sendCommand(0x14);
	sendCommand(MEMORYMODE);
	sendCommand(0x00);
	sendCommand(SEGREMAP|0x01);
	sendCommand(COMSCANDEC);
	sendCommand(SETCOMPINS);

    sendCommand(0x12);
    sendCommand(SETCONTRAST);

    sendCommand(0xCF);

    sendCommand(SETPRECHARGE);
    sendCommand(0xF1);
    sendCommand(SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
    sendCommand(0x40);	        //0x40 default, to lower the contrast, put 0
    sendCommand(DISPLAYALLON_RESUME);
    sendCommand(NORMALDISPLAY);
    sendCommand(0x2e);            // stop scroll
    sendCommand(DISPLAYON);
}

enum OLEDDISPLAY_COLOR
{
	BLACK = 0,
	WHITE = 1,
	INVERSE = 2
};

typedef enum OLEDDISPLAY_COLOR OLEDDISPLAY_COLOR;

void setPixelColor(int16_t x, int16_t y, OLEDDISPLAY_COLOR color)
{
	switch (color)
	{
      case WHITE:   _buffer[x + (y >> 3) * _displayWidth ] |=  (1 << (y & 7)); break;
      case BLACK:   _buffer[x + (y >> 3) * _displayWidth ] &= ~(1 << (y & 7)); break;
      case INVERSE: _buffer[x + (y >> 3) * _displayWidth ] ^=  (1 << (y & 7)); break;
    }
}

uint8_t* fontData = ArialMT_Plain_16;


void drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData, OLEDDISPLAY_COLOR color)
{
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > _displayHeight)  return;
  if (xMove + width  < 0 || xMove > _displayWidth )   return;

  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;


  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }

    uint8_t currentByte = *(data + offset + i);

    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * _displayWidth;

    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < _displayBufferSize &&
        xPos    >=  0  && xPos    < _displayWidth ) {

      if (yOffset >= 0) {
        switch (color) {
          case WHITE:   _buffer[dataPos] |= currentByte << yOffset; break;
          case BLACK:   _buffer[dataPos] &= ~(currentByte << yOffset); break;
          case INVERSE: _buffer[dataPos] ^= currentByte << yOffset; break;
        }

        if (dataPos < (_displayBufferSize - _displayWidth)) {
          switch (color) {
            case WHITE:   _buffer[dataPos + _displayWidth] |= currentByte >> (8 - yOffset); break;
            case BLACK:   _buffer[dataPos + _displayWidth] &= ~(currentByte >> (8 - yOffset)); break;
            case INVERSE: _buffer[dataPos + _displayWidth] ^= currentByte >> (8 - yOffset); break;
          }
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        switch (color) {
          case WHITE:   _buffer[dataPos] |= currentByte >> yOffset; break;
          case BLACK:   _buffer[dataPos] &= ~(currentByte >> yOffset); break;
          case INVERSE: _buffer[dataPos] ^= currentByte >> yOffset; break;
        }

        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }
    }
  }
}


void drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth, OLEDDISPLAY_COLOR color)
{
  uint8_t textHeight       = *(fontData + HEIGHT_POS);
  uint8_t firstChar        = *(fontData + FIRST_CHAR_POS);
  uint16_t sizeOfJumpTable = *(fontData + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

  uint16_t cursorX         = 0;
  uint16_t cursorY         = 0;

  // Don't draw anything if it is not on the screen.
  if (xMove + textWidth  < 0 || xMove > _displayWidth ) {return;}
  if (yMove + textHeight < 0 || yMove > _displayHeight ) {return;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;

    uint8_t code = text[j];
    if (code >= firstChar) {
      uint8_t charCode = code - firstChar;

      // 4 Bytes per char code
      uint8_t msbJumpToChar    = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
      uint8_t lsbJumpToChar    = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
      uint8_t charByteSize     = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
      uint8_t currentCharWidth = *( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        drawInternal(xPos, yPos, currentCharWidth, textHeight, fontData, charDataPosition, charByteSize, color);
      }

      cursorX += currentCharWidth;
    }
  }
}

uint16_t getStringWidth(const char* text, uint16_t length)
{
  uint16_t firstChar        = *(fontData + FIRST_CHAR_POS);

  uint16_t stringWidth = 0;
  uint16_t maxWidth = 0;

  while (length--) {
    stringWidth += *(fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
    if (text[length] == 10) {
      maxWidth = fmax(maxWidth, stringWidth);
      stringWidth = 0;
    }
  }

  return fmax(maxWidth, stringWidth);
}

void drawString(int16_t xMove, int16_t yMove, char *stringUser, OLEDDISPLAY_COLOR color )
{
  uint16_t lineHeight = *(fontData + HEIGHT_POS);

  uint16_t yOffset = 0;
  uint16_t line = 0;

  char* textPart = strtok(stringUser,"\n");
  while (textPart != NULL) {
    uint16_t length = strlen(textPart);
    drawStringInternal(xMove, yMove - yOffset + (line++) * lineHeight, textPart, length, getStringWidth(textPart, length), color);
    textPart = strtok(NULL, "\n");
  }

}


void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    _displayWidth = 128;
    _displayHeight = 64;
    _displayBufferSize = _displayWidth * _displayHeight/8;

    _buffer = (uint8_t*) malloc((sizeof(uint8_t) * _displayBufferSize) );
  	_buffer_back = (uint8_t*) malloc((sizeof(uint8_t) * _displayBufferSize) );
    memset(_buffer, 0, _displayBufferSize);
    memset(_buffer_back, 0, _displayBufferSize);


    initializeReset( RESET_PIN );
    initializeOLED();

	printf( "Clearing Display\n");
    memset(_buffer, 0, _displayBufferSize);
    sendData();
    vTaskDelay( 500 / portTICK_PERIOD_MS);

    int col = 0;

    OLEDDISPLAY_COLOR color = WHITE;

    for ( ;; )
    {
    	char buf[100];

    	memset(_buffer, 0, _displayBufferSize);
    	//sprintf( buf, "Count: 09001" );
    	sprintf( buf, "Count: %06d\nOther: %06d", col, col*col );

    	drawString( 10, 10, buf, color );

/*
    	for ( int i = 0 ; i < 64 ; i++ )
        	setPixelColor( i+col, i, color );
*/
    	//sendData();
    	sendDataBack();
        col++;

        //vTaskDelay( 1000 / portTICK_PERIOD_MS);
    }
}


