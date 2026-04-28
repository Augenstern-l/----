#include "take.h"
#include "oled.h" 
#include "codetab.h"



extern I2C_HandleTypeDef hi2c2;

uint8_t OLED_GRAM[128][8];
uint8_t OLED_GRAMbuf[8][128];
uint8_t OLED_CMDbuf[8][4] = {0};
uint8_t I2C1_MemTxFinshFlag = 1;
uint8_t CountFlag = 0; 
uint8_t BufFinshFlag = 0; 



/**
  * @brief    HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
              uint16_t Size)完成回调函数
  * 					保证DMA传输完成后，开启下次DMA
  */
	
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(BufFinshFlag)
	{
		HAL_I2C_Mem_Write_DMA(&hi2c2,0x78,0x40,I2C_MEMADD_SIZE_8BIT,OLED_GRAMbuf[CountFlag],128);
	}
}


/**
  * @brief    HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
              uint16_t MemAddSize, uint8_t *pData, uint16_t Size)完成回调函数
  * 					保证DMA传输完成后，开启下次DMA
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(CountFlag == 7)
	{
		BufFinshFlag = 0;
		CountFlag = 0;
	}
	if(BufFinshFlag)
	{
		CountFlag ++;
		HAL_I2C_Master_Transmit_DMA(&hi2c2,0x78,OLED_CMDbuf[CountFlag],4);
	}
}




uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;

  while (n--)
    result *= m;

  return result;
}

uint8_t check_num_len(uint32_t num)
{
  uint32_t tmp;
	uint8_t i;
  for(i = 1; i < 10; i++)
  {
    tmp = oled_pow(10, i);
    if(num < tmp)
    {
      return i;
    }
  }
	return 0;
}

/**
 * oled显示数字
 * x
 * y
 * num：要显示的数字
 * mode：显示模式
 * len：数字长度
 * size：大小
*/
void OLED_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t mode, uint8_t len, uint8_t size)
{
  uint8_t t, temp;
  uint8_t enshow = 0;

  for (t = 0; t < len; t++)
  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;

    if (enshow == 0 && t < (len - 1))
    {
      if (temp == 0)
      {
        if (mode == 0)
          OLED_show_char(x, y + t, ' ',size);
        else
          OLED_show_char(x, y + t, '0',size);
        continue;
      }
      else
        enshow = 1;
    }

    OLED_show_char(x, y + t, temp + '0',size);
  }
}

void OLED_show_floatnum(uint8_t x, uint8_t y, float num, uint8_t mode, uint8_t size)
{
  int32_t m, n;
  float R;
  uint8_t chartemp[6], i;
	
  R = num;
  m = R / 1;
  n = (R - m) * 1000;
  if(n < 0) n = -n;

  i = check_num_len(m);

  OLED_show_num(x, y, m, mode, i,size);

  chartemp[0] = '.';
  chartemp[1] = n / 100 + 48;
  chartemp[2] = n / 10 % 10 + 48;
  chartemp[3] = n % 10 + 48;
  chartemp[4] = ' ';
  chartemp[5] = '\0';
  OLED_show_string(x, y + i, chartemp,size);
}

/**
  * @brief          initialize the oled device
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化OLED模块，
  * @param[in]      none
  * @retval         none
  */
uint8_t OLED_Init_CMD[ ] =
{
	0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8,
	0x3F, 0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1,
	0xDA, 0x12, 0xDB, 0x30, 0x8D, 0x14, 0xAF, 0x20, 0x00
//    0xAE, // 显示关闭（睡眠模式）
//    0x20, 0x00, // 设置内存地址模式
//    0x00, // 设置低列地址
//    0x10, // 设置高列地址
//    0x40, // 设置起始行地址
//    0xB0, // 设置页地址模式的页起始地址
//    0x81, 0xFF, // 设置对比度控制寄存器
//    0xA1, // 段重映射：列地址 127 映射到 SEG0（翻转）
//    0xC8, // COM 输出扫描方向：扫描从 COM[N-1] 到 COM0
//    0xA6, // 正常显示
//    0xA8, 0x3F, // 设置多路复用比率（1 到 64）
//    0xD3, 0x00, // 设置显示偏移
//    0xD5, 0xF0, // 设置显示时钟分频比/振荡器频率
//    0xD9, 0x22, // 设置预充电周期
//    0xDA, 0x12, // 设置 COM 引脚硬件配置
//    0xDB, 0x20, // 设置 VCOMH 取消选择级别
//    0x8D, 0x14, // 启用充电泵调节器
//    0xAF // 显示开启
};

void OLED_init(void)
{
	HAL_Delay(200);
	HAL_I2C_Mem_Write_DMA(&hi2c2, OLED_I2C_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, OLED_Init_CMD, 29);
}

/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM内存(128*8char数组)
  * @param[in]      pen: 操作类型.
                    PEN_CLEAR: 设置为0x00
                    PEN_WRITE: 设置为0xff
  * @retval         none
  */
void OLED_operate_gram(pen_typedef pen)
{
	if (pen == PEN_WRITE)
	{
			memset(OLED_GRAM,0xff,sizeof(OLED_GRAM));
	}
	else if(pen == PEN_CLEAR)
	{
			memset(OLED_GRAM,0x00,sizeof(OLED_GRAM));
	}
	
}

/**
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          设置光标起点(x,y)
  * @param[in]      x:x轴, 从 0 到 127
  * @param[in]      y:y轴, 从 0 到 7
  * @retval         none
  */
void OLED_set_pos(uint8_t x, uint8_t y)
{
	OLED_CMDbuf[y][0] = 0x00;
	OLED_CMDbuf[y][1] = 0xb0 + y;
	OLED_CMDbuf[y][2] = 0x10;
	OLED_CMDbuf[y][3] = 0x00;
	
}


/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM中的一个位，相当于操作屏幕的一个点
  * @param[in]      x:x轴,  [0,X_WIDTH-1]
  * @param[in]      y:y轴,  [0,Y_WIDTH-1]
  * @param[in]      pen: 操作类型,
                        PEN_CLEAR: 设置 (x,y) 点为 0
                        PEN_WRITE: 设置 (x,y) 点为 1
                        PEN_INVERSION: (x,y) 值反转
  * @retval         none
  */
void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        OLED_GRAM[x][page] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        OLED_GRAM[x][page] ^= 1 << row;
    }
    else
    {
        OLED_GRAM[x][page] &= ~(1 << row);
    }
}




/**
  * @brief          draw a line from (x1, y1) to (x2, y2)
  * @param[in]      x1: the start point of line
  * @param[in]      y1: the start point of line
  * @param[in]      x2: the end point of line
  * @param[in]      y2: the end point of line
  * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
/**
  * @brief          画一条直线，从(x1,y1)到(x2,y2)
  * @param[in]      x1: 起点
  * @param[in]      y1: 起点
  * @param[in]      x2: 终点
  * @param[in]      y2: 终点
  * @param[in]      pen: 操作类型,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
  
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}


/**
  * @brief          show a character
  * @param[in]      row: start row of character
  * @param[in]      col: start column of character
  * @param[in]      chr: the character ready to show
  * @retval         none
  */
/**
  * @brief          显示一个字符
  * @param[in]      row: 字符的开始行
  * @param[in]      col: 字符的开始列
  * @param[in]      chr: 字符
  * @retval         none
  */
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr, uint8_t size)
{
	const unsigned char *bitmap; // 获取对应数字的字节数组
	
    uint8_t x = col * (size/2);
    uint8_t y = row * size;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
	
	if (size == 12)
	{
		chr = chr - ' ';	//偏移量

		for (t = 0; t < 12; t++)
		{
			temp = asc2_1206[chr][t];

			for (t1 = 0; t1 < 8; t1++)
			{
				if (temp&0x80)
					OLED_draw_point(x, y, PEN_WRITE);
				else
					OLED_draw_point(x, y, PEN_CLEAR);

				temp <<= 1;
				y++;
				if ((y - y0) == 12)
				{
					y = y0;
					x++;
					break;
				}
			}
		}
	}
	else if (size == 32)
	{
		
		if(chr == ' ')
		{
			bitmap = &asc2_3216[0][0]; // 获取对应数字的字节数组
		}
		else
		{
			bitmap = &asc2_3216[(chr)* 4 + 4][0]; // 获取对应数字的字节数组
		}

		for (t = 0; t < 4; t++) // 每个数字占用4个8x16块，总共32行
		{
			for (t1 = 0; t1 < 16; t1++) // 每块16列
			{
				temp = bitmap[t * 16 + t1]; // 获取字节

				for (uint8_t bit = 0; bit < 8; bit++) // 每个字节8位
				{
					if (temp & 0x80)
						OLED_draw_point(x, y, PEN_WRITE);
					else
						OLED_draw_point(x, y, PEN_CLEAR);

					temp <<= 1;
					y++;
					if ((y - y0) == 32) // 到达32行后回到初始行，列加1
					{
						y = y0;
						x++;
						break;
					}
				}
			}
		}
	}
}

void OLED_draw_char_rolling(uint8_t x, uint8_t y, char chr, char old_chr, uint8_t size)
{
    const uint8_t *bitmap, *old_bitmap;
    uint8_t t, t1;
    uint8_t temp;
    uint8_t y0 = y;
    uint8_t diff;
    int16_t stop, i;

    if (size == 32)
	{
		//				  chr*8 0~3
		bitmap = &asc2_3216[chr][0]; // 获取对应数字的字节数组

		for (t = 0; t < 4; t++) // 每个数字占用4个8x16块，总共32行
		{

			for (t1 = 0; t1 < 16; t1++) // 每块16列
			{
				temp = bitmap[t * 16 + t1]; // 获取字节

				for (uint8_t bit = 0; bit < 8; bit++) // 每个字节8位
				{
					if (temp & 0x80)
						OLED_draw_point(x, y, PEN_WRITE);
					else
						OLED_draw_point(x, y, PEN_CLEAR);

					temp <<= 1;
					y++;
					if ((y - y0) == 32) // 到达32行后回到初始行，列加1	竖向
					{
						y = y0;
						x++;
						break;
					}
				}
			}
			
		}
	}
}

/**
  * @brief          show a character string
  * @param[in]      row: row of character string begin
  * @param[in]      col: column of character string begin
  * @param[in]      chr: the pointer to character string
  * @retval         none
  */
/**
  * @brief          显示一个字符串
  * @param[in]      row: 字符串的开始行
  * @param[in]      col: 字符串的开始列
  * @param[in]      chr: 字符串
  * @retval         none
  */
void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr, uint8_t size)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(row, col, chr[n],size);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}

// ---------------------- 核心：16×16汉字显示函数 ----------------------
/**
 * @brief  显示16×16点阵汉字
 * @param  row: 汉字起始行（0~3，16行/个，64行屏幕可显示4行）
 * @param  col: 汉字起始列（0~7，16列/个，128列屏幕可显示8个）
 * @param  hz_idx: 汉字索引（时=0，间=1）
 */
void OLED_show_chinese_16x16(uint8_t row, uint8_t col, uint8_t hz_idx)
{
   uint8_t i, j;
    uint8_t x = col * 16;
    uint8_t y = row * 16;
    uint8_t temp;

    /* 上半部分 8 行 */
    for (i = 0; i < 16; i++)
    {
        temp = chinese_1616[hz_idx * 2][i];
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
                OLED_draw_point(x + i, y + j, PEN_WRITE);
            else
                OLED_draw_point(x + i, y + j, PEN_CLEAR);

            temp <<= 1;
        }
    }

    /* 下半部分 8 行 */
    for (i = 0; i < 16; i++)
    {
        temp = chinese_1616[hz_idx * 2 + 1][i];
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
                OLED_draw_point(x + i, y + j + 8, PEN_WRITE);
            else
                OLED_draw_point(x + i, y + j + 8, PEN_CLEAR);

            temp <<= 1;
        }
    }
}

/**
  * @brief          formatted output in oled 128*64
  * @param[in]      row: row of character string begin, 0 <= row <= 4;
  * @param[in]      col: column of character string begin, 0 <= col <= 20;
  * @param          *fmt: the pointer to format character string
  * @note           if the character length is more than one row at a time, the extra characters will be truncated
  * @retval         none
  */
/**
  * @brief          格式输出
  * @param[in]      row: 开始列，0 <= row <= 4;
  * @param[in]      col: 开始行， 0 <= col <= 20;
  * @param[in]      *fmt:格式化输出字符串
  * @note           如果字符串长度大于一行，额外的字符会换行
  * @retval         none
  */
void OLED_printf(uint8_t row, uint8_t col,uint8_t size, const char *fmt,...)
{
    static uint8_t LCD_BUF[128] = {0};
    static va_list ap;
    uint8_t remain_size = 0;

    if ((row > 4) || (col > 20) )
    {
        return;
    }
    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    OLED_show_string(row, col, LCD_BUF,size);
}

/**
  * @brief          send the data of gram to oled sreen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送数据到OLED的GRAM
  * @param[in]      none
  * @retval         none
  */
void OLED_refresh_gram(void)
{	
	uint8_t i;
	uint16_t j;
		
	if(BufFinshFlag == 0)
	{
		for(i = 0; i < 8; i ++ )
		{
			OLED_set_pos(0, i);
			for(j = 0;j < 128; j ++)
			{
					OLED_GRAMbuf[i][j] = OLED_GRAM[j][i];  //OLED_GRAM[128][8]
			}
		}
		BufFinshFlag = 1;
		HAL_I2C_Master_Transmit_DMA(&hi2c2,0x78,OLED_CMDbuf[0],4);
	}
}



/**
  * @brief          show the logo of RoboMaster
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          显示RM的LOGO
  * @param[in]      none
  * @retval         none
  */
//void OLED_LOGO(void)
//{
//    uint8_t temp_char = 0;
//    uint8_t x = 0, y = 0;
//    uint8_t i = 0;
//    OLED_operate_gram(PEN_CLEAR);


//    for(; y < 64; y += 8)
//    {
//        for(x = 0; x < 128; x++)
//        {
//            temp_char = LOGO_BMP[x][y/8];
//            for(i = 0; i < 8; i++)
//            {
//                if(temp_char & 0x80)
//                {
//                    OLED_draw_point(x, y + i,PEN_WRITE);
//                }
//                else
//                {
//                    OLED_draw_point(x,y + i,PEN_CLEAR);
//                }
//                temp_char <<= 1;
//            }
//        }
//    }
//    OLED_refresh_gram();
//}

