#ifndef __BOARD_INCLUDES_H__
#define __BOARD_INCLUDES_H__



#define VERSION_2_0 0

#define VERSION_2_1 1

#define VERSION_2_2 2


/// Define the Board Here
#define BOARD_VERSION VERSION_2_2

#if BOARD_VERSION == VERSION_2_2

    // #pragma message ("Board Version 2.2")
    #define DebugPort Serial1

#else

    #pragma message("Board Version 2.1/2.0")
    #define DebugPort Serial

#endif


#endif //  __BOARD_INCLUDES_H__
