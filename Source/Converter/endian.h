#ifndef ENDIAN_H
#define ENDIAN_H

#include <stdint.h>

#if (defined(__arm__) && !defined(__ARMEB__)) || defined(__i386__) || defined(__i860__) || defined(__ns32000__) || defined(__vax__) || defined(__amd64__) || defined(__x86_64__)
#undef WORDS_BIGENDIAN
#elif defined(__sparc__) || defined(__alpha__) || defined(__PPC__) || defined(__mips__) || defined(__ppc__) || defined(__BIG_ENDIAN__)
#define WORDS_BIGENDIAN 1
#endif

#define swap16(x) ((((uint16_t)(x) & 0x00ff)<<8)| \
	(((uint16_t)(x) & 0xff00)>>8))
#define swap32(x) ((((uint32_t)(x) & 0x000000ff)<<24)| \
	(((uint32_t)(x) & 0x0000ff00)<<8)| \
	(((uint32_t)(x) & 0x00ff0000)>>8)| \
	(((uint32_t)(x) & 0xff000000)>>24))

#ifdef WORDS_BIGENDIAN
#define h2be16(x) (x)
#define h2be32(x) (x)
#define h2le16(x) swap16(x)
#define h2le32(x) swap32(x)
#else
#define h2le16(x) (x)
#define h2le32(x) (x)
#define h2be16(x) swap16(x)
#define h2be32(x) swap32(x)
#endif

#endif
