
#include "pwmdriver.hpp"

namespace motorlib
{

pwmdriver::pwmdriver()
    : parent(nullptr)
{}

pwmdriver::~pwmdriver()
{}

void pwmdriver::link_timer(pwm_callback_memptr cb, VVVF* _parent)
{
    this->callback_ptr = cb;
    this->parent = _parent;
}

void pwmdriver::invoke_callbacks(int pwm, int perids)
{
    if (parent)
        (parent->*callback_ptr)(pwm, perids);
}

}
