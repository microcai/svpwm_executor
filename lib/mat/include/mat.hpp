
#pragma once

#include <array>

namespace matrix
{

template<typename T, int Row, int Colum>
using mat = std::array<std::array<T, Colum>, Row>;

}
