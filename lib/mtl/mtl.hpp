
// mcu template library

namespace mtl {

    template <typename T>
    class static_store
    {
        int storage[sizeof(T)/sizeof(int) + ((sizeof(T) % sizeof(int)) ? 1 : 0)];

    public:
        void* address()
        {
            static_assert(sizeof(storage) >= sizeof(T) );
            return &storage[0];
        }
    };

}