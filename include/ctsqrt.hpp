
#pragma once

#include "type_traits"

namespace ct
{
    static constexpr int sqrt( int num )
    {
        int bit { ( 1 ) << ( sizeof( int ) * 8 - 2 ) };

        while ( bit > num )
            bit >>= 2;

        int res{ 0 };
        while ( bit )
        {
            int delta{ res + bit };
            if ( num >= delta )
            {
                num -= delta;
                res = ( res >> 1 ) + bit;
            }
            else
            {
                res >>= 1;
            }
            bit >>= 2;
        }
        return res;
    }
}