/******************************************************************************
SparkFun_Alphanumeric_Display_Pico_Animation.h
SparkFun Alphanumeric Display Animation Library Raspberry Pico Header File

This file is for Raspberry Pi PICO 
Development environment specifics:
	Hardware Platform: Raspberry Pi
	Alphanumeric Display Breakout Version: 1.0.0


Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __SparkFun_Alphanumeric_Display_Animation_Pico_H__
#define __SparkFun_Alphanumeric_Display_Animation_Pico_H__

#include "SparkFun_Alphanumeric_Display_Pico.h"


#define MAX_OUTER_BOX_SEGMENTS ((4*4*2)+(2+2))

typedef struct _entry_table_t
{
    uint8_t digit;
    char    segment;       
} entry_table_t;

class SparkFun_Alphanumeric_Display_Animations: public  HT16K33
{
    private:
           uint32_t delay = 125;
           
           // Variables related to building arrows.
           bool     directionLeftToRight = true;
           bool     buildArrow = true;
           int8_t digit = 0;
           int8_t arrowHeadDigit = 0;
           //  Variables for an outer box
           char outer_box_cleared_segment = 'A';
           uint8_t outer_box_cleared_digit = 0;
           entry_table_t entry_table[MAX_OUTER_BOX_SEGMENTS];
           uint8_t index;
           uint8_t table_max_entries = 0;
           // Draw outer box on each digit 
           char segmentSkipped[16];


    public:
       // Functions to build arrows
       void animate_Arrow(void);
       void drawLeftArrowHead(uint8_t digit);
       void clearLeftArrowHead(uint8_t digit);
       void drawRightArrowHead(uint8_t digit);
       void clearRightArrowHead(uint8_t digit);
       void drawArrowTail(uint8_t digit);
       void clearArrowTail(uint8_t digit);
       void draw_rightArrow(void);
       void draw_leftArrow(void);
       void clear_rightArrow(void);
       void clear_leftArrow(void);
       void draw_Arrow(void);
       //  Functions to draw an outer arrow
       void outer_box_clear_segment(uint8_t table_index);
       void outer_box_illuminate_segment(uint8_t table_index);

       void outer_box_setup_table(void);
       void build_outer_box(void);
       void animate_outer_box(void);
       
       // Draw outer box on each digit
       void setup_digit_outer_box(uint8_t digit);
       void setup_digits_outer_box(void);
       void animate_digit_outer_box(uint8_t digit);
       void animate_digits_outer_box(void);
       void animate_digit_outer_box(void);

};

#endif // __SparkFun_Alphanumeric_Display_Animation_Pico_H__