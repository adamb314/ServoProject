#include <memory>
#include <cmath>

#include "DummyTrajectoryGenerator.h"
#include "RobotParameters.h"

#ifndef PATH_AND_MOVE_BUILDER_H
#define PATH_AND_MOVE_BUILDER_H

class VelocityLimiter
{
public:
    VelocityLimiter(const double& velocity, const EigenVectord6& selector = EigenVectord6::Ones(),
            const double& distFromBendAcc = 0.0);

    void add(const double& velocity, const EigenVectord6& selector = EigenVectord6::Ones(),
            const double& distFromBendAcc = 0.0);

    void setMoveDir(const EigenVectord6& moveDir);

    double getLimit(const double& distFromBend = 0.0) const;

private:
    class Zip
    {
    public:
        double velocity{0.0};
        double distFromBendAcc{0.0};
        EigenVectord6 selector{EigenVectord6::Ones()};
        double selectorScalingResult{1.0};
    };

    std::vector<Zip> limits;
};

class DeviationLimiter
{
public:
    virtual double getLimit(const EigenVectord6& pos, const EigenVectord6& preDir, const EigenVectord6& postDir) const = 0;
};

class JointSpaceDeviationLimiter : public DeviationLimiter
{
public:
    JointSpaceDeviationLimiter(const double& deviation, const EigenVectord6& selector = EigenVectord6::Ones());

    void add(const double& deviation, const EigenVectord6& selector);

    virtual double getLimit(const EigenVectord6& pos, const EigenVectord6& preDir, const EigenVectord6& postDir) const;

protected:
    class Zip
    {
    public:
        double deviation{0};
        EigenVectord6 selector{EigenVectord6::Ones()};
    };

    std::vector<Zip> limits;
};

class CartesianSpaceDeviationLimiter : public JointSpaceDeviationLimiter
{
public:
    CartesianSpaceDeviationLimiter(const double& deviation, const EigenVectord6& selector = EigenVectord6{1.0, 1.0, 1.0, 0.0, 0.0, 0.0});

    virtual double getLimit(const EigenVectord6& pos, const EigenVectord6& preDir, const EigenVectord6& postDir) const override;
};

class PathObjectInterface
{
public:
    class BendItem
    {
    public:
        EigenVectord6 pos;
        double requestedVelForLink;
        double requestedVelAtBend;
        std::shared_ptr<DeviationLimiter> deviationLimiter;
    };

    class IteratorInterface
    {
    public:
        IteratorInterface(const void* id);

        IteratorInterface(const IteratorInterface& in);

        virtual ~IteratorInterface() = default;

        void step();

        bool operator==(const IteratorInterface& in);

        virtual BendItem getBendItem() = 0;

        virtual void stepImp() = 0;

        virtual std::unique_ptr<IteratorInterface> makeCopy() = 0;

    protected:
        void setEnd();

    private:
        size_t index;
        size_t id;
    };

    class Iterator
    {
    public:
        Iterator(std::unique_ptr<IteratorInterface>& in);

        Iterator(const Iterator& in);

        BendItem operator*();

        Iterator& operator++();

        Iterator operator++(int);

        bool operator==(Iterator& in);

        bool operator!=(Iterator& in);

        Iterator& operator=(const Iterator& in);

    private:
        std::unique_ptr<IteratorInterface> it;
    };

    virtual ~PathObjectInterface() = default;

    virtual Iterator begin() const = 0;
    virtual Iterator end() const = 0;

private:
    virtual void setStart(const EigenVectord6& pos) = 0;

    friend class PathAndMoveBuilder;
};

class PathAndMoveBuilder
{
public:
    PathAndMoveBuilder();

    ~PathAndMoveBuilder() = default;

    void clear();

    void append(std::unique_ptr<PathObjectInterface>&& object);

    void renderTo(DummyTrajectoryGenerator& trajectoryGenerator, const EigenVectord6& startPos);

private:
    std::vector<std::shared_ptr<PathObjectInterface> > objects;
};

RobotParameters::DynamicMatrices getDynamicMatrices(const EigenVectord6& a);

class CartesianCoordinate;

class JointSpaceCoordinate
{
public:
    EigenVectord6 c = {};

    JointSpaceCoordinate() = default;

    JointSpaceCoordinate(const EigenVectord6& pos) :
            c(pos)
    {
    }

    JointSpaceCoordinate(const CartesianCoordinate& cartesian);
};

class CartesianCoordinate
{
public:
    EigenVectord6 c = {};

    CartesianCoordinate() = default;

    CartesianCoordinate(const EigenVectord6& pos) :
            c(pos)
    {
    }

    CartesianCoordinate(const JointSpaceCoordinate& joint);
};

class JointSpaceLinearPath : public PathObjectInterface
{
public:
    static std::unique_ptr<JointSpaceLinearPath> create(const JointSpaceCoordinate& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    static std::unique_ptr<JointSpaceLinearPath> create(const EigenVectord6& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    JointSpaceLinearPath(const EigenVectord6& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    virtual ~JointSpaceLinearPath() = default;

    class Iterator : public PathObjectInterface::IteratorInterface
    {
    public:
        Iterator(const JointSpaceLinearPath* parent);

        Iterator(const Iterator& in);

        virtual ~Iterator() = default;

        virtual PathObjectInterface::BendItem getBendItem();

        virtual void stepImp();

        virtual std::unique_ptr<IteratorInterface> makeCopy();

    private:
        void init();

        void updateCurrentBendItem();

        const JointSpaceLinearPath* parent;
        double requestedVel;
        double requestedVelAtEnd;
        double t;
        double stepSize;
        PathObjectInterface::BendItem bendItem;

        friend class JointSpaceLinearPath;
    };

    virtual PathObjectInterface::Iterator begin() const;

    virtual PathObjectInterface::Iterator end() const;

private:
    virtual void setStart(const EigenVectord6& pos);

    JointSpaceCoordinate startPos;
    JointSpaceCoordinate endPos;
    VelocityLimiter velocityLimiter;
    VelocityLimiter velocityLimiterAtEnd;
    std::shared_ptr<DeviationLimiter> deviationLimiter;
};

class CartesianSpaceLinearPath : public PathObjectInterface
{
public:
    static std::unique_ptr<CartesianSpaceLinearPath> create(const CartesianCoordinate& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    static std::unique_ptr<CartesianSpaceLinearPath> create(const EigenVectord6& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    CartesianSpaceLinearPath(const EigenVectord6& pos,
            VelocityLimiter velocityLimiter,
            VelocityLimiter velocityLimiterAtEnd,
            std::shared_ptr<DeviationLimiter> deviationLimiter);

    virtual ~CartesianSpaceLinearPath() = default;

    class Iterator : public PathObjectInterface::IteratorInterface
    {
    public:
        Iterator(const CartesianSpaceLinearPath* parent);

        Iterator(const Iterator& in);

        virtual ~Iterator() = default;

        virtual PathObjectInterface::BendItem getBendItem();

        virtual void stepImp();

        virtual std::unique_ptr<IteratorInterface> makeCopy();

    private:
        void init();

        void updateCurrentBendItem();

        const CartesianSpaceLinearPath* parent;
        double requestedVel;
        double requestedVelAtEnd;
        double t;
        double stepSize;
        PathObjectInterface::BendItem bendItem;

        friend class CartesianSpaceLinearPath;
    };

    virtual PathObjectInterface::Iterator begin() const;

    virtual PathObjectInterface::Iterator end() const;

private:
    virtual void setStart(const EigenVectord6& pos);

    CartesianCoordinate startPos;
    CartesianCoordinate endPos;
    VelocityLimiter velocityLimiter;
    VelocityLimiter velocityLimiterAtEnd;
    std::shared_ptr<DeviationLimiter> deviationLimiter;
};

#endif
