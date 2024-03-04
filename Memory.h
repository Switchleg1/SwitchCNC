#ifndef MEMORY_H
#define MEMORY_H


inline void memcopy2(void* dest, void* source) {
	*((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void* dest, void* source) {
	*((int32_t*)dest) = *((int32_t*)source);
}

#endif