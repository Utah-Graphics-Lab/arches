#pragma once

#include "int.hpp"

namespace rtm {

template<uint BITS>
struct BitArray
{
    uint8_t data[(BITS + 7) / 8];
    
    uint64_t read(uint index, uint size) const
    {
        if(size == 0) return 0;
        uint64_t background_data = 0;
        uint64_t mask = (0x1ull << size) - 1;

        uint shft = index & 0x7;
        uint start_byte = index >> 3;
        uint last_bit = index + size - 1;
        uint end_byte = (last_bit >> 3) + 1;

        const uint8_t* cpy_ptr = data + start_byte;
        uint cpy_size = end_byte - start_byte;
        for(uint i = 0; i < cpy_size; ++i)
            background_data |= ((uint64_t)cpy_ptr[i]) << (i << 3);

        return (background_data >> shft) & mask;
    }
    
    void write(uint index, uint size, uint64_t value)
    {
        if(size == 0) return;
        uint64_t background_data = 0;
        uint64_t mask = (0x1ull << size) - 1;
        value = value & mask;

        uint shft = index & 0x7;
        uint start_byte = index >> 3;
        uint last_bit = index + size - 1;
        uint end_byte = (last_bit >> 3) + 1;

        uint8_t* cpy_ptr = data + start_byte;
        uint cpy_size = end_byte - start_byte;
        for(uint i = 0; i < cpy_size; ++i)
            background_data |= ((uint64_t)cpy_ptr[i]) << (i << 3);
        
        background_data &= ~(mask << shft);
        background_data |= value << shft;

        for(uint i = 0; i < cpy_size; ++i)
            cpy_ptr[i] = (uint8_t)(background_data >> (i << 3));
    }
};

inline uint log2i(uint64_t u)
{
    if(u == 0) return 0;
    for(uint i = 0; i < 64; ++i)
        if((u >> i) == 0) return (i - 1);
    return 63;
}
    
}