#include "app.h"
#include "test_tx.h"

void App_SetUp(void) { Test_TxIt_SetUp(); }
void App_Loop(void) { Test_TxIt_Run(); }
