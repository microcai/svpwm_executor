
#pragma once


template<typename HolderType, HolderType min, HolderType max>
struct cyc_counter
{
    static_assert(max > min, "max should bigger than min");

    static HolderType constexpr numbercount = (max - min);
    static HolderType constexpr halfcount = (numbercount)/2;

    cyc_counter(){};
    cyc_counter(HolderType init) : holded(init){}

    void operator ++(){
        holded++;
        range_fix();
    }

    void operator += (HolderType delta){
        holded += delta;
        range_fix();
    }

    void operator --(){
        holded --;
        range_fix();
    }

    void operator -= (HolderType delta){
        holded -= delta;
        range_fix();
    }

    HolderType operator - (cyc_counter other){
        // 寻找差异，正差异 or 负差异.
        bool cur_bigger = holded >= other.holded;
        auto diff = holded - other.holded;

        if (cur_bigger)
        {
            if (diff > halfcount)
            {
                return diff - max;
            }
            return diff;
        }
        else
        {
            if ( (-diff) > (halfcount))
            {
                return max + diff;
            }
            return diff;
        }
    }

    bool operator > (cyc_counter other)
    {
        auto diff = holded - other.holded;
        if (diff > halfcount)
        {
            return false;
        }
        else if (diff > 0)
        {
            return true;
        }
        else if (diff > -halfcount)
        {
            return false;
        }
        return true;
    }

    HolderType holded;

    void range_fix()
    {
        while (holded >= max)
            holded -= numbercount;
        while (holded < min)
            holded += numbercount;
    }

};

