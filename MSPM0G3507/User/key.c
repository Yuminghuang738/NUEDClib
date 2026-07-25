#include "key.h"

uint8_t get_key_state(uint8_t id)
{
    switch (id)
    {
        case 1:
        {
            uint32_t high_bits = DL_GPIO_readPins(KEY_KEY_1_PORT ,KEY_KEY_1_PIN );
            
            if ((high_bits & KEY_KEY_1_PIN) != 0)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        case 2:
        {
            uint32_t high_bits = DL_GPIO_readPins(KEY_KEY_2_PORT ,KEY_KEY_2_PIN );
            
            if ((high_bits & KEY_KEY_2_PIN) != 0)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        case 3:
        {
            uint32_t high_bits = DL_GPIO_readPins(KEY_KEY_3_PORT ,KEY_KEY_3_PIN );
            
            if ((high_bits & KEY_KEY_3_PIN) != 0)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        case 4:
        {
            uint32_t high_bits = DL_GPIO_readPins(KEY_KEY_2_PORT ,KEY_KEY_2_PIN );
        
            if((high_bits & KEY_KEY_4_PIN) != 0)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        default:
        {    
            return 0;
        }
    }
}