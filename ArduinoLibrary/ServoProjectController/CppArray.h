#ifndef CPP_ARRAY_H
#define CPP_ARRAY_H

extern void indexOutOfBoundsErrorHandler(size_t index, size_t limit);

template <typename T, size_t N>
class CppArray
{
public:
    using iterator = T*;
    using const_iterator = const T*;

    CppArray() = default;
    CppArray(const CppArray &) = default;
    CppArray(CppArray &&) = default;
    ~CppArray() = default;
    CppArray& operator = (const CppArray &) = default;
    CppArray& operator = (CppArray &&) = default;

    T* begin() {return cArray;}
    T* end() {return cArray + N;}
    const T* cbegin() const {return cArray;}
    const T* cend() const {return cArray + N;}
    const T* begin() const {return cbegin();}
    const T* end() const {return cend();}

    T& operator[] (const size_t& i) {return cArray[i];}
    const T& operator[] (const size_t& i) const {return cArray[i];}

    T& at(const size_t& i)
    {
        if (i >= N)
        {
            indexOutOfBoundsErrorHandler(i, N);
            terminateExecution();
        }
        return cArray[i];
    }

    const T& at(const size_t& i) const
    {
        if (i >= N)
        {
            indexOutOfBoundsErrorHandler(i, N);
            terminateExecution();
        }
        return cArray[i];
    }

    T* data() {return cArray;}
    const T* data() const {return cArray;}

    static constexpr size_t size() {return N;}

    template<typename V>
    void fill(const V& v)
    {
        for (auto& d : cArray)
        {
            d = v;
        }
    }

    T cArray[N];

private:
    static void terminateExecution()
    {
        while (true);
    }
};

#endif
