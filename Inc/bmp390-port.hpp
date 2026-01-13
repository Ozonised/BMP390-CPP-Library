/*
 * bmp390-port.hpp
 *
 *  Created on: Jan 13, 2026
 *      Author: farhan
 */

#ifndef BMP390_PORT_HPP_
#define BMP390_PORT_HPP_

#include <stdint.h>

#ifndef BMP390_RET_TYPE
typedef uint8_t BMP390_RET_TYPE;
#endif

#ifndef BMP390_RET_TYPE_SUCCESS
#define BMP390_RET_TYPE_SUCCESS 0
#endif

#ifndef BMP390_RET_TYPE_FAILURE
#define BMP390_RET_TYPE_FAILURE 1
#endif

#ifndef BMP390_RET_TYPE_BUSY
#define BMP390_RET_TYPE_BUSY 2
#endif

#endif /* BMP390_PORT_HPP_ */
