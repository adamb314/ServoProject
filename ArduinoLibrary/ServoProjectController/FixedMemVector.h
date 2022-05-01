#include "stdmin.h"
#include <new>

#ifndef FIXED_MEM_VECTOR_H
#define FIXED_MEM_VECTOR_H

extern void indexOutOfBoundsErrorHandler(size_t index, size_t limit);
extern void outOfMemErrorHandler(size_t limitReached);

template <typename T, size_t N>
class FixedMemVector
{
public:
    using iterator = T*;
    using const_iterator = const T*;

    FixedMemVector()
    {
    }

    template <typename V>
    FixedMemVector(const stdmin::remove_reference<V>& in)
    {
        (*this) = in;
    }

    template <typename V>
    FixedMemVector(stdmin::remove_reference<V>&& in)
    {
        (*this) = stdmin::move(in);
    }

    ~FixedMemVector()
    {
        clear();
    }

    template <typename V>
    FixedMemVector& operator = (const stdmin::remove_reference<V>& in)
    {
        clear();
        for (auto& d : in)
        {
            push_back(d);
        }
        return *this;
    }

    template <typename V>
    FixedMemVector& operator = (stdmin::remove_reference<V>&& in)
    {
        clear();
        for (auto&& d : in)
        {
            push_back(stdmin::move(d));
        }
        return *this;
    }

    T* begin() {return getMemAsT();}
    T* end() {return getMemAsT() + nrOfElements;}
    const T* cbegin() const {return getMemAsT();}
    const T* cend() const {return getMemAsT() + nrOfElements;}
    const T* begin() const {return cbegin();}
    const T* end() const {return cend();}

    T& operator[] (const size_t& i) {return getMemAsT()[i];}
    const T& operator[] (const size_t& i) const {return getMemAsT()[i];}

    T& at(const size_t& i)
    {
        if (i >= nrOfElements)
        {
            indexOutOfBoundsErrorHandler(i, nrOfElements);
            terminateExecution();
        }
        return getMemAsT()[i];
    }

    const T& at(const size_t& i) const
    {
        if (i >= nrOfElements)
        {
            indexOutOfBoundsErrorHandler(i, nrOfElements);
            terminateExecution();
        }
        return getMemAsT()[i];
    }

    T* data() {return getMemAsT();}
    const T* data() const {return getMemAsT();}

    size_t size() const {return nrOfElements;}
    static constexpr size_t max_size() {return N;}

    void push_back(const T& v)
    {
        if (nrOfElements >= N)
        {
            outOfMemErrorHandler(nrOfElements);
            terminateExecution();
        }

        ::new (getMemAsT() + nrOfElements) T(v);
        ++nrOfElements;
    }

    void push_back(T&& v)
    {
        if (nrOfElements >= N)
        {
            outOfMemErrorHandler(nrOfElements);
            terminateExecution();
        }

        ::new (getMemAsT() + nrOfElements) T(stdmin::move(v));
        ++nrOfElements;
    }

    void clear()
    {
        for (auto it = begin(); it != end(); ++it)
        {
            callDestructor(it);
        }
        nrOfElements = 0;
    }

    size_t nrOfElements{0};

private:
    static void terminateExecution()
    {
        while (true);
    }

    class alignas(T) MemSeg
    {
        char data[sizeof(T)];
    };

    T* getMemAsT()
    {
        return reinterpret_cast<T*>(mem);
    }

    const T* getMemAsT() const
    {
        return reinterpret_cast<const T*>(mem);
    }

    static void callDestructor(T* p)
    {
        p->~T();
    }

    MemSeg mem[N];
};
#endif
