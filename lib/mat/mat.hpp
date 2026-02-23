
#pragma once

#include <initializer_list>

template<int M, int K, typename Number_t = float>
struct matrix
{
	Number_t values[M][K];

	constexpr matrix(){}

	constexpr matrix(std::initializer_list<std::initializer_list<Number_t>> init)
	{
		// static_assert((init.begin() - init.end()) == M, "array size should match");
		for (int i=0;i<M; i++)
			for(int j=0; j < K; j++)
				values[i][j] = * ((init.begin()+ i)->begin() +j );
	}

	template<int N>
	matrix<M, N> operator *(const matrix<K, N, Number_t> & other) const
	{
		matrix<M, N> ret;

		for (int i=0; i< M; i++)
		{
			for (int j=0; j < N; j++)
			{
				Number_t row_sum = 0;
				for (int r=0; r < K ; r++ )
				{
					row_sum += this->values[i][r] * other.values[r][j];
				}

				ret.values[i][j] = row_sum;
			}
		}

		return ret;
	}
};
