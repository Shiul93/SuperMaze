#include "utils.h"


/** 
 * @brief  Returns the sign of a value
 * @note   
 * @param  x: the value
 * @retval 1 if > 0 else -1
 */
int sign(int x) {
    return (x > 0) - (x < 0);
  }