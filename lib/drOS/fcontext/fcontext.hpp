
#pragma once

typedef void*   fcontext_t;

struct transfer_t {
    fcontext_t  fctx;
    void    *   data;
};

extern "C" fcontext_t make_fcontext( void * sp, unsigned size, void (* fn)( transfer_t) );
extern "C" transfer_t jump_fcontext( fcontext_t const to, void * vp);

// based on an idea of Giovanni Derreta
extern "C" transfer_t ontop_fcontext( fcontext_t const to, void * vp, transfer_t (* fn)( transfer_t) );
