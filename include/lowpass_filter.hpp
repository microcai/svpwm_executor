#pragma once

/**
 *  Low pass filter class
 */
template<typename FLOAT = float>
class LowPass_Filter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    LowPass_Filter(FLOAT time_constant = 0.0f)
        : Tf(time_constant)
        , y_prev(0)
    {
    }

    ~LowPass_Filter() = default;

    FLOAT update(FLOAT x, FLOAT dt)
    {
        if (dt < 0.0f ) dt = 1e-3f;
        else if(dt > 0.3f) {
            y_prev = x;
            return x;
        }

        FLOAT alpha = Tf;
        alpha /= (Tf + dt);
        FLOAT y = alpha*y_prev + (FLOAT{1} - alpha)*x;
        y_prev = y;
        return y;            
    }    

    FLOAT operator() (float x) { return update(x); };
    FLOAT Tf; //!< Low pass filter time constant

    FLOAT getOutput() const { return y_prev;}

protected:
    FLOAT y_prev; //!< filtered value in previous execution step 
};
