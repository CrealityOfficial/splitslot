#ifndef SPLITSLOT_INTERFACE_1604911737496_H
#define SPLITSLOT_INTERFACE_1604911737496_H
#include "ccglobal/export.h"

#ifdef SPLITSLOT_DLL
	#define SPLITSLOT_API CC_DECLARE_EXPORT
#else
	#define SPLITSLOT_API CC_DECLARE_IMPORT
#endif
#endif // SPLITSLOT_INTERFACE_1604911737496_H