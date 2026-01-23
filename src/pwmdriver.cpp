
#include "pwmdriver.hpp"

namespace motorlib
{

pwmdriver::pwmdriver()
    : callback_ptr(nullptr)
{}

pwmdriver::~pwmdriver()
{}

void pwmdriver::link_timer(pwm_callback_ptr cb, void* _user_data)
{
    this->callback_ptr = cb;
    this->callback_ptr_user_data = _user_data;
}

void pwmdriver::unlink_timer()
{
    this->callback_ptr = nullptr;
    this->callback_ptr_user_data = nullptr;    
}

void pwmdriver::invoke_callbacks(int pwm, int perids)
{
    if (callback_ptr)
        callback_ptr(pwm, perids, callback_ptr_user_data);
}

}
