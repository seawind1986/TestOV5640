#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdlib.h>
#include <stdint.h>

#define LWIP_RAND() ((u32_t)rand())

#define LWIP_PLATFORM_ASSERT(message) do { (void)(message); } while (0)
#define LWIP_PLATFORM_DIAG(message) do { } while (0)

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#endif
