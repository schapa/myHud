//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

#include "diag/Trace.h"

// ----------------------------------------------------------------------------

void
__attribute__((noreturn))
__assert_func (const char *file, int line, const char *func,
               const char *failedexpr)
{
  trace_printf ("assertion \"%s\" failed: file \"%s\", line %d%s%s\n",
                failedexpr, file, line, func ? ", function: " : "",
                func ? func : "");
  abort ();
  /* NOTREACHED */
}


#if defined(USE_FULL_ASSERT)

void assert_failed (uint8_t* file, uint32_t line);

// Called from the assert_param() macro, usually defined in the stm32f*_conf.h
void __attribute__((noreturn))
assert_failed (uint8_t* file, uint32_t line)
{
  trace_printf ("assert_param() failed: file \"%s\", line %d\n", file, line);
  abort ();
  /* NOTREACHED */
}

#endif // defined(USE_FULL_ASSERT)

// ----------------------------------------------------------------------------
