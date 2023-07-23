#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

class ComplementaryFilter
{
public:
    static constexpr long int wrapAroundSize = 4096;
    static constexpr long int fixedPoint = 512;
    ComplementaryFilter(float x0 = 0.0f);

    void update(float lowFrqIn, float highFrqIn);

    float get();

    void set(float x0);

    void setFilterConst(float a);

private:
    long int aFixed{0};
    long int lastHighFrqInFixed;
    long int outFixed;
    long int outWrapAround{0};
};

#endif
