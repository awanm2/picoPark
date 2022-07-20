/******************************************************************************
SparkFun_Alphanumeric_Display_Animations.cpp
SparkFun Alphanumeric Display Animation Library Source File


Current File is for Raspberry Pi Pico.

This file implements animation functions for Spartkfun Alphanumeric Displays,which uses HT16K33.

Development environment specifics:
	Hardware Platform: Raspberry Pi Pico
	Alphanumeric Display Breakout Version: 1.0.0


Distributed as-is; no warranty is given.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include "SparkFun_Alphanumeric_Display_Animation_Pico.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"


void SparkFun_Alphanumeric_Display_Animations::drawLeftArrowHead(uint8_t digit)
{
   this->illuminateSegment('K',  digit);
   this->illuminateSegment('L',  digit);
}

void SparkFun_Alphanumeric_Display_Animations::clearLeftArrowHead(uint8_t digit)
{
   this->clearSegment('K',  digit);
   this->clearSegment('L',  digit);
}


void SparkFun_Alphanumeric_Display_Animations::drawRightArrowHead(uint8_t digit)
{
   this->illuminateSegment('H',  digit);
   this->illuminateSegment('N',  digit);
}

void SparkFun_Alphanumeric_Display_Animations::clearRightArrowHead(uint8_t digit)
{
   this->clearSegment('H',  digit);
   this->clearSegment('N',  digit);
}


void SparkFun_Alphanumeric_Display_Animations::drawArrowTail(uint8_t digit)
{
   this->illuminateSegment('G',  digit);
   this->illuminateSegment('I',  digit);
}


void SparkFun_Alphanumeric_Display_Animations::clearArrowTail(uint8_t digit)
{
   this->clearSegment('G',  digit);
   this->clearSegment('I',  digit);
}

void SparkFun_Alphanumeric_Display_Animations::draw_rightArrow(void)
{
   
   if( digit == 0)
   {
      this->clearLeftArrowHead(this->arrowHeadDigit);
      this->drawRightArrowHead(this->digit);
   }
   else
   {
      this->clearRightArrowHead(this->arrowHeadDigit);
      this->drawArrowTail(this->arrowHeadDigit);
       this->drawRightArrowHead(this->digit);
   }
   this->arrowHeadDigit = this->digit;
   this->digit = (this->digit+1) % (numberOfDisplays * DIGIT_PER_DISPLAY);
      
   if(this->digit == 0)
   {
      buildArrow = false;
   }
      
}

// clear arrow from left to right 
void SparkFun_Alphanumeric_Display_Animations::clear_rightArrow(void)
{
   
   if(this->arrowHeadDigit == this->digit)
   {
      this->clearLeftArrowHead(this->arrowHeadDigit);
   }
   else
   {
      this->clearArrowTail(this->digit);
   }

   this->digit = (this->digit+1) % (numberOfDisplays * DIGIT_PER_DISPLAY);
      
   if(this->digit == 0)
   {
      directionLeftToRight = false;
      buildArrow = true;
      this->digit = this->arrowHeadDigit;
      
   }
      
}

// Draw arrow from right to left

void SparkFun_Alphanumeric_Display_Animations::draw_leftArrow(void)
{
   
   
   if( digit == ((numberOfDisplays * DIGIT_PER_DISPLAY)-1))
   {
      this->clearRightArrowHead(this->arrowHeadDigit);
      this->drawLeftArrowHead(this->digit);
   }
   else
   {
      this->clearLeftArrowHead(this->arrowHeadDigit);
      this->drawArrowTail(this->arrowHeadDigit);
       this->drawLeftArrowHead(this->digit);
   }

   this->arrowHeadDigit = this->digit;
   this->digit = (this->digit+ ((numberOfDisplays * DIGIT_PER_DISPLAY))-1) % (numberOfDisplays * DIGIT_PER_DISPLAY);

   if(this->arrowHeadDigit == 0)
   {
      buildArrow = false;
   
   }
}

void SparkFun_Alphanumeric_Display_Animations::clear_leftArrow(void)
{

   if(this->arrowHeadDigit == this->digit)
   {
      this->clearLeftArrowHead(this->arrowHeadDigit);
   }
   else
   {
      this->clearArrowTail(this->digit);
   }
   
   this->digit = (this->digit+ ((numberOfDisplays * DIGIT_PER_DISPLAY))-1) % (numberOfDisplays * DIGIT_PER_DISPLAY);
   
   if(this->digit == 0)
   {
      this->buildArrow = true;
      this->digit = this->arrowHeadDigit;
      this->directionLeftToRight = true;
   }

}

void SparkFun_Alphanumeric_Display_Animations::draw_Arrow(void)
{
   if(directionLeftToRight)
   {
      if(buildArrow)
      {
         this->draw_rightArrow();
      }
      else
      {
         this->clear_rightArrow();
      }
   }
   else
   {
      if(buildArrow)
      {
         this->draw_leftArrow();
      }
      else
      {
         this->clear_leftArrow();
      }   
   }
}


void SparkFun_Alphanumeric_Display_Animations::animate_Arrow(void)
{    
    static volatile bool runForever = true;
    while(runForever)
    {
        draw_Arrow();
        this->updateDisplay();
        sleep_ms(this->delay);
    }
    
}


void SparkFun_Alphanumeric_Display_Animations::animate_outer_box(void)
{    
    static volatile bool runForever = true;
    this->outer_box_setup_table();
    while(runForever)
    {
        this->updateDisplay();
        this->build_outer_box();
        sleep_ms(this->delay);
    }
    
}

// Functions for a animating outer box on the display
void SparkFun_Alphanumeric_Display_Animations::outer_box_clear_segment(uint8_t table_index)
{
   entry_table_t * entry;
   entry = &(this->entry_table[table_index]);
   this->clearSegment(entry->segment, entry->digit);    
}

void SparkFun_Alphanumeric_Display_Animations::outer_box_illuminate_segment(uint8_t table_index)
{
   entry_table_t * entry;
   entry = &(this->entry_table[table_index]);
   this->illuminateSegment(entry->segment, entry->digit);
  
}

void SparkFun_Alphanumeric_Display_Animations::outer_box_setup_table(void)
{
   uint8_t k = 0;
   entry_table_t * entry;

   for (uint8_t i = 0; i < numberOfDisplays; i++)
   {
      for (uint8_t j = 0; j < DIGIT_PER_DISPLAY; j++)
      {
         entry = &(this->entry_table[k++]);
         entry->digit = (i*DIGIT_PER_DISPLAY)+ j;
         entry->segment = 'A';
         this->illuminateSegment(entry->segment, entry->digit);
      }
   }

   entry = &(this->entry_table[k++]);
   entry->digit = (numberOfDisplays*DIGIT_PER_DISPLAY)-1;
   entry->segment = 'B';
   this->illuminateSegment(entry->segment, entry->digit);

   entry = &(this->entry_table[k++]);
   entry->digit = (numberOfDisplays*DIGIT_PER_DISPLAY)-1;
   entry->segment = 'C';
   this->illuminateSegment(entry->segment, entry->digit);
   
   for (int8_t i = numberOfDisplays-1; i >= 0; i--)
   {
      for (int8_t j = DIGIT_PER_DISPLAY-1; j >= 0; j--)
      {
         entry = &(this->entry_table[k++]);
         entry->digit = (i*DIGIT_PER_DISPLAY)+ j;
         entry->segment = 'D';
         this->illuminateSegment(entry->segment, entry->digit);      
      }
   }

   entry = &(this->entry_table[k++]);
   entry->digit = 0;
   entry->segment = 'E';
   this->illuminateSegment(entry->segment, entry->digit);

   entry = &(this->entry_table[k++]);
   entry->digit = 0;
   entry->segment = 'F';
   this->illuminateSegment(entry->segment, entry->digit);

   this->table_max_entries = k;
   this->index = 0;
   this->outer_box_clear_segment(this->index);
 
}

void SparkFun_Alphanumeric_Display_Animations::build_outer_box(void)
{
   this->outer_box_illuminate_segment(this->index);
   this->index = (this->index +1 ) %    this->table_max_entries;
   this->outer_box_clear_segment(this->index);
}

// Functions to draw outer box on each digit
void SparkFun_Alphanumeric_Display_Animations::setup_digit_outer_box(uint8_t digit)
{
   char segment = 'A';
   while(segment < 'G')
   {
       if((segment - 'A') != (digit%6))
       {
           this->illuminateSegment(segment,  digit);
       }
       else
       {
            // Which segment is skipped
            this->segmentSkipped[digit] = segment - 'A';
       }
       segment++;
   } 
}

void SparkFun_Alphanumeric_Display_Animations::setup_digits_outer_box(void)
{
   	for (uint8_t i = 0; i < numberOfDisplays; i++)
    {
        for (uint8_t j = 0; j < DIGIT_PER_DISPLAY; j++)
        {
            this->setup_digit_outer_box( j +(i*DIGIT_PER_DISPLAY));
        }
    }
}

void SparkFun_Alphanumeric_Display_Animations::animate_digit_outer_box(uint8_t digit)
{
   this->illuminateSegment(this->segmentSkipped[digit] + 'A',  digit);
   this->segmentSkipped[digit] = ( this->segmentSkipped[digit] + 1 ) % 6;
   this->clearSegment(this->segmentSkipped[digit] + 'A',  digit);

}



// This animates each outer digit
void SparkFun_Alphanumeric_Display_Animations::animate_digits_outer_box(void)
{

   	for (uint8_t i = 0; i < numberOfDisplays; i++)
    {
        for (uint8_t j = 0; j < DIGIT_PER_DISPLAY; j++)
        {
            this->animate_digit_outer_box( j +(i*DIGIT_PER_DISPLAY));
        }
    }

}

// Animate a rotating circle on each digit.
void SparkFun_Alphanumeric_Display_Animations::animate_digit_outer_box(void)
{
   	
    this->setup_digits_outer_box();  
    this->updateDisplay();
    sleep_ms(this->delay);
    static volatile bool runForever = true;
    while(runForever)
    {
        this->animate_digits_outer_box();
        this->updateDisplay();
        sleep_ms(this->delay);
    }
}
