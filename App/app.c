#include "app.h"

#ifdef CC_ROLE_ARM
#include "arm_app.h"

void App_SetUp(void) { Arm_Setup(); }
void App_Loop(void) { Arm_Loop(); }

#else
#include "master_app.h"

void App_SetUp(void) { Master_Setup(); }
void App_Loop(void) { Master_Loop(); }

#endif
