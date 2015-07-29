//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <vector>
#include "diag/Trace.h"

#include "stm32f10x.h"
#include "bsp.hpp"

#include "subaruCANMessaes.hpp"
#include "bspSTM32.hpp"
using namespace subaruCAN;
using namespace BoardSupportPackageSpace;

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"




int main(int argc, char* argv[]){

	CanMessage *messages[] = {new MessageESP, new MessageStatusInfo, new MessageIGNITION, new MessageFUEL};
	for(unsigned int i=0; i < sizeof(messages)/sizeof(messages[0]); i++)
		messages[i]->sendMessage();


	BoardSupportPackageSTM32& aa = BoardSupportPackageSTM32::getInstance();


	extern void catTest(void);
	catTest();
	while(1);

	return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
