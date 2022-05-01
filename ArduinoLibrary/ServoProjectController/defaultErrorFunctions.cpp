#include <Arduino.h>

void __attribute__((weak)) indexOutOfBoundsErrorHandler(size_t index, size_t limit)
{
}

void __attribute__((weak)) outOfMemErrorHandler(size_t limitReached)
{
}
