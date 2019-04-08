#include "ArduinoC++BugFixes.h"

namespace std
{
    void __throw_bad_function_call()
    {
    }

    void __throw_bad_alloc()
    {
    }
}