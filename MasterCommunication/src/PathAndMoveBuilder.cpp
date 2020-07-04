#include "PathAndMoveBuilder.h"

#include <numeric>

using namespace RobotParameters;

VelocityLimiter::VelocityLimiter(const double& velocity, const EigenVectord6& selector, const double& distFromBendAcc)
{
    add(velocity, selector, distFromBendAcc);
}

void VelocityLimiter::add(const double& velocity, const EigenVectord6& selector, const double& distFromBendAcc)
{
    limits.push_back(Zip{velocity, distFromBendAcc, selector});
}

void VelocityLimiter::setMoveDir(const EigenVectord6& moveDir)
{
    const double& moveDirNorm = moveDir.norm();

    std::for_each(std::begin(limits), std::end(limits),
        [&moveDirNorm, &moveDir](auto& z) {
            const double& selectedMoveDirNorm = (z.selector.asDiagonal() * moveDir).norm();

            z.selectorScalingResult = moveDirNorm / selectedMoveDirNorm;
        });
}

double VelocityLimiter::getLimit(const double& distFromBend) const
{
    return std::accumulate(std::cbegin(limits), std::cend(limits), std::numeric_limits<double>::max(),
        [&distFromBend](const double& currentMin, const auto& z) {
            return std::min(currentMin,
                    (z.velocity + z.distFromBendAcc * distFromBend) * z.selectorScalingResult);
        });
}

JointSpaceDeviationLimiter::JointSpaceDeviationLimiter(const double& deviation, const EigenVectord6& selector)
{
    add(deviation, selector);
}

void JointSpaceDeviationLimiter::add(const double& deviation, const EigenVectord6& selector)
{
    limits.push_back(Zip{deviation, selector});
}

double JointSpaceDeviationLimiter::getLimit(const EigenVectord6& pos, const EigenVectord6& preDir, const EigenVectord6& postDir) const
{
    const EigenVectord6& deviationDir = (postDir - preDir) / 2.0;

    const double& deviationDirNorm = deviationDir.norm();

    return std::accumulate(std::cbegin(limits), std::cend(limits), std::numeric_limits<double>::max(),
        [&deviationDirNorm, &deviationDir](const double& currentMin, const auto& z) {
            const double& selectedDeviationDirNorm = (z.selector.asDiagonal() * deviationDir).norm();

            return std::min(currentMin, z.deviation * deviationDirNorm / selectedDeviationDirNorm);
        });
}

CartesianSpaceDeviationLimiter::CartesianSpaceDeviationLimiter(const double& deviation, const EigenVectord6& selector) :
        JointSpaceDeviationLimiter(deviation, selector)
{
}

double CartesianSpaceDeviationLimiter::getLimit(const EigenVectord6& pos, const EigenVectord6& preDir, const EigenVectord6& postDir) const
{
    const EigenVectord6& deviationDir = 0.001 * (postDir - preDir) / 2.0;

    const double& deviationDirNorm = deviationDir.norm();

    const CartesianCoordinate temp1{JointSpaceCoordinate{pos}};
    const CartesianCoordinate temp2{JointSpaceCoordinate{pos + deviationDir}};
    const EigenVectord6 cartesianDeviationDir = temp1.c - temp2.c;

    return std::accumulate(std::cbegin(limits), std::cend(limits), std::numeric_limits<double>::max(),
        [&deviationDirNorm, &cartesianDeviationDir](const double& currentMin, const auto& z) {
            const double& selectedDeviationDirNorm = (z.selector.asDiagonal() * cartesianDeviationDir).norm();

            return std::min(currentMin, z.deviation * deviationDirNorm / selectedDeviationDirNorm);
        });
}

PathObjectInterface::IteratorInterface::IteratorInterface(const void* id)
{
    this->id = reinterpret_cast<size_t>(id);

    index = 0;
}

PathObjectInterface::IteratorInterface::IteratorInterface(const IteratorInterface& in)
{
    id = in.id;
    index = in.index;
}

void PathObjectInterface::IteratorInterface::step()
{
    if (index == static_cast<size_t>(-1))
    {
        return;
    }

    index++;
    stepImp();
}

bool PathObjectInterface::IteratorInterface::operator==(const IteratorInterface& in)
{
    return (index == in.index) && (id == in.id);
}

void PathObjectInterface::IteratorInterface::setEnd()
{
    index = -1;
}

PathObjectInterface::Iterator::Iterator(std::unique_ptr<PathObjectInterface::IteratorInterface>& in) :
    it{std::move(in)}
{
}

PathObjectInterface::Iterator::Iterator(const Iterator& in) :
    it{in.it->makeCopy()}
{
}

PathObjectInterface::BendItem PathObjectInterface::Iterator::operator*()
{
    return it->getBendItem();
}

PathObjectInterface::Iterator& PathObjectInterface::Iterator::operator++()
{
    it->step();

    return (*this);
}

PathObjectInterface::Iterator PathObjectInterface::Iterator::operator++(int)
{
    Iterator out(*this);
    ++(*this);
    return out;
}

bool PathObjectInterface::Iterator::operator==(Iterator& in)
{
    return (*it) == (*in.it);
}

bool PathObjectInterface::Iterator::operator!=(Iterator& in)
{
    return !((*this) == in);
}

PathObjectInterface::Iterator& PathObjectInterface::Iterator::operator=(const Iterator& in)
{
    it = in.it->makeCopy();
    return *this;
}

PathAndMoveBuilder::PathAndMoveBuilder()
{
}

void PathAndMoveBuilder::clear()
{
    objects.clear();
}

void PathAndMoveBuilder::append(std::unique_ptr<PathObjectInterface>&& object)
{
    objects.push_back(std::move(object));
}

void PathAndMoveBuilder::renderTo(DummyTrajectoryGenerator& trajectoryGenerator, const EigenVectord6& startPos)
{
    PathObjectInterface::BendItem bend0{startPos, 0, 0, nullptr};
    PathObjectInterface::BendItem bend1 = bend0;
    PathObjectInterface::BendItem bend2 = bend0;

    bool allInit = false;

    for (auto& objectPtr : objects)
    {
        auto& object = *objectPtr.get();
        object.setStart(bend2.pos);

        for (const auto& bendItem : object)
        {
            bend0 = bend1;
            bend1 = bend2;
            bend2 = bendItem;

            if (allInit)
            {
                EigenVectord6 v0 = bend1.pos - bend0.pos;
                EigenVectord6 v1 = bend2.pos - bend1.pos;

                v0.normalize();
                v1.normalize();

                double maxDeviation = bend1.deviationLimiter->getLimit(bend1.pos, v0, v1);

                trajectoryGenerator.addBendPoint(bend1.pos,
                        bend1.requestedVelForLink,
                        bend1.requestedVelAtBend,
                        maxDeviation);
            }
            else
            {
                allInit = true;
            }
        }
    }

    bend1 = bend2;
    trajectoryGenerator.addBendPoint(bend1.pos,
            bend1.requestedVelForLink,
            bend1.requestedVelAtBend,
            std::numeric_limits<double>::max());
}

JointSpaceCoordinate::JointSpaceCoordinate(const CartesianCoordinate& cartesian)
{
    const double& xTransLength = s1Translation[0] + s2Translation[0] + s3Translation[0];
    const EigenVectord3 xTrans(xTransLength, 0 , 0);

    EigenVectord3 s1s2s3Vec;
    s1s2s3Vec << cartesian.c[0], cartesian.c[1], cartesian.c[2];    

    EigenVectord3 s4s5Vec = s5Rotation(cartesian.c[4]) * s5Translation;
    s4s5Vec = s1Rotation(cartesian.c[3]) * s4s5Vec;

    EigenVectord3 s6Vec = s6Rotation(cartesian.c[5]) * s6ZeroRotationDir;
    s6Vec = s5Rotation(cartesian.c[4]) * s6Vec;
    s6Vec = s1Rotation(cartesian.c[3]) * s6Vec;

    s1s2s3Vec -= s4s5Vec;

    const double& x = s1s2s3Vec[0];
    const double& y = s1s2s3Vec[1];
    const double& z = s1s2s3Vec[2];

    double xy = sqrt(x * x + y * y);

    if (xy < xTransLength)
    {
        xy = xTransLength;
    }

    c[0] = atan2(y, x);
    c[0] += acos(xTransLength / xy);

    EigenMatrixd3 r1M = s1Rotation(-c[0]);

    EigenVectord3 temp = r1M * EigenVectord3(x, y, z) - s1Translation;

    const double& yp = temp[1];
    const double& zp = temp[2];

    const double& yzp2 = yp * yp + zp * zp;
    const double& yzp = sqrt(yzp2);

    const double& ap1 = atan2(zp, yp);

    const double& s2L = -s2Translation[1];
    const double& s3L = -s3Translation[1];

    const double& ap2 = acos((s2L * s2L + yzp2 - s3L * s3L) / (2 * s2L * yzp));
    const double& bp = acos((s2L * s2L + s3L * s3L - yzp2) / (2 * s2L * s3L));

    c[1] = M_PI - ap1 + ap2;
    c[2] = M_PI - bp;

    EigenMatrixd3 r2M = s2Rotation(-c[1]);
    EigenMatrixd3 r3M = s3Rotation(-c[2]);

    EigenVectord3 s4s5VecNormalized = r3M * r2M * r1M * s4s5Vec;

    s4s5VecNormalized.normalize();

    double cosAngle5 = (-ey).dot(s4s5VecNormalized);
    double angle5 = acos(cosAngle5);

    EigenVectord3 s4s5XZ = s4s5VecNormalized + ey * cosAngle5;
    s4s5XZ.normalize();
    if (s4s5XZ[2] < 0)
    {
        angle5 *= -1.0;
        s4s5XZ *= -1.0;
    }

    double angle4 = asin(s4s5XZ[0]);

    EigenMatrixd3 r4M = s4Rotation(-angle4);
    EigenMatrixd3 r5M = s5Rotation(-angle5);

    s6Vec = r5M * r4M * r3M * r2M * r1M * s6Vec;

    double sinAngle6 = ex.dot(s6Vec);
    double angle6 = -asin(sinAngle6);

    c[3] = angle4;
    c[4] = angle5;
    c[5] = angle6;
}

CartesianCoordinate::CartesianCoordinate(const JointSpaceCoordinate& joint)
{
    EigenMatrixd3 r6M = s6Rotation(joint.c[5]);
    EigenMatrixd3 r5M = s5Rotation(joint.c[4]);
    EigenMatrixd3 r4M = s4Rotation(joint.c[3]);
    EigenMatrixd3 r3M = s3Rotation(joint.c[2]);
    EigenMatrixd3 r2M = s2Rotation(joint.c[1]);
    EigenMatrixd3 r1M = s1Rotation(joint.c[0]);

    EigenVectord3 s6RotDir = r1M * r2M * r3M * r4M * r5M * r6M * s6ZeroRotationDir;
    EigenVectord3 s4s5Vec = r1M * r2M * r3M * r4M * (s4Translation +  r5M * s5Translation);
    EigenVectord3 s1s2s3Vec = r1M * (s1Translation + r2M * (s2Translation + r3M * s3Translation));

    EigenVectord3 s4s5VecNormalized = s4s5Vec;
    s4s5VecNormalized.normalize();

    EigenVectord3 s6sinProjectionVec = ez.cross(s4s5VecNormalized);

    s6sinProjectionVec.normalize();

    double sinAngle3 = s6sinProjectionVec.dot(s6RotDir);

    double angle3 = -asin(sinAngle3);

    double sinAngle2 = ez.dot(s4s5VecNormalized);

    double angle2 = asin(sinAngle2);

    double angle1 = atan2(s6sinProjectionVec[1], s6sinProjectionVec[0]);

    EigenVectord3 xyzPos = s1s2s3Vec + s4s5Vec;

    c << xyzPos[0], xyzPos[1], xyzPos[2], angle1, angle2, angle3;
}

std::unique_ptr<JointSpaceLinearPath> JointSpaceLinearPath::create(const JointSpaceCoordinate& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter)
{
    return std::unique_ptr<JointSpaceLinearPath>(
            new JointSpaceLinearPath(pos.c, std::move(velocityLimiter), std::move(velocityLimiterAtEnd), std::move(deviationLimiter)));
}

std::unique_ptr<JointSpaceLinearPath> JointSpaceLinearPath::create(const EigenVectord6& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter)
{
    return std::unique_ptr<JointSpaceLinearPath>(
            new JointSpaceLinearPath(pos, std::move(velocityLimiter), std::move(velocityLimiterAtEnd), std::move(deviationLimiter)));
}

JointSpaceLinearPath::JointSpaceLinearPath(const EigenVectord6& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter) :
    startPos{},
    endPos{pos},
    velocityLimiter{std::move(velocityLimiter)},
    velocityLimiterAtEnd{std::move(velocityLimiterAtEnd)},
    deviationLimiter{std::move(deviationLimiter)}
{
}

JointSpaceLinearPath::Iterator::Iterator(const JointSpaceLinearPath* parent) :
    PathObjectInterface::IteratorInterface{parent},
    parent{parent},
    t{},
    stepSize{},
    bendItem{}
{
    init();
}

JointSpaceLinearPath::Iterator::Iterator(const Iterator& in) :
    PathObjectInterface::IteratorInterface{in},
    parent{in.parent},
    t{in.t},
    stepSize{in.stepSize},
    bendItem{in.bendItem}
{
}

void JointSpaceLinearPath::Iterator::init()
{
    const EigenVectord6&  a = parent->startPos.c;
    const EigenVectord6&  b = parent->endPos.c;

    const EigenVectord6 v = (b - a);
    const double vNorm = v.norm(); 

    stepSize = 0.002 / vNorm;

    size_t nrOfSteps = static_cast<size_t>(ceil(1.0 / stepSize));
    stepSize = 1.0 / nrOfSteps;

    requestedVelAtEnd = parent->velocityLimiterAtEnd.getLimit();
}

PathObjectInterface::BendItem JointSpaceLinearPath::Iterator::getBendItem()
{
    return bendItem;
}

void JointSpaceLinearPath::Iterator::stepImp()
{
    if (t == 1.0)
    {
        PathObjectInterface::IteratorInterface::setEnd();
        return;
    }

    t += stepSize;

    if (t >= 1.0 - stepSize * 0.5)
    {
        t = 1.0;
    }

    updateCurrentBendItem();
}

void JointSpaceLinearPath::Iterator::updateCurrentBendItem()
{
    const EigenVectord6&  a = parent->startPos.c;
    const EigenVectord6&  b = parent->endPos.c;

    const EigenVectord6 v = (b - a);
    const double vNorm = v.norm(); 
    JointSpaceCoordinate current{a + t * v};

    bendItem.pos = current.c;
    bendItem.requestedVelForLink = parent->velocityLimiter.getLimit(
            (std::min(t, 1.0 - t) + stepSize) * vNorm);

    if (t != 1.0)
    {
        bendItem.requestedVelAtBend = bendItem.requestedVelForLink;
    }
    else
    {
        bendItem.requestedVelAtBend = requestedVelAtEnd;
    }

    bendItem.deviationLimiter = parent->deviationLimiter;
}

std::unique_ptr<PathObjectInterface::IteratorInterface> JointSpaceLinearPath::Iterator::makeCopy()
{
    return std::make_unique<Iterator>(*this);
}

PathObjectInterface::Iterator JointSpaceLinearPath::begin() const
{
    auto it = std::make_unique<Iterator>(this);
    it->updateCurrentBendItem();
    it->stepImp();

    auto out = std::unique_ptr<PathObjectInterface::IteratorInterface>(std::move(it));

    return PathObjectInterface::Iterator(out);
}

PathObjectInterface::Iterator JointSpaceLinearPath::end() const
{
    auto it = std::make_unique<Iterator>(this);
    it->setEnd();
    auto out = std::unique_ptr<PathObjectInterface::IteratorInterface>(std::move(it));

    return PathObjectInterface::Iterator(out);
}

void JointSpaceLinearPath::setStart(const EigenVectord6& pos)
{
    startPos = JointSpaceCoordinate(pos);

    const EigenVectord6 v = (endPos.c - startPos.c);

    velocityLimiter.setMoveDir(v);
    velocityLimiterAtEnd.setMoveDir(v);
}

std::unique_ptr<CartesianSpaceLinearPath> CartesianSpaceLinearPath::create(const CartesianCoordinate& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter)
{
    return std::unique_ptr<CartesianSpaceLinearPath>(
            new CartesianSpaceLinearPath(pos.c, std::move(velocityLimiter), std::move(velocityLimiterAtEnd), std::move(deviationLimiter)));
}

std::unique_ptr<CartesianSpaceLinearPath> CartesianSpaceLinearPath::create(const EigenVectord6& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter)
{
    return std::unique_ptr<CartesianSpaceLinearPath>(
            new CartesianSpaceLinearPath(pos, std::move(velocityLimiter), std::move(velocityLimiterAtEnd), std::move(deviationLimiter)));
}

CartesianSpaceLinearPath::CartesianSpaceLinearPath(const EigenVectord6& pos,
        VelocityLimiter velocityLimiter,
        VelocityLimiter velocityLimiterAtEnd,
        std::shared_ptr<DeviationLimiter> deviationLimiter) :
    startPos{},
    endPos{pos},
    velocityLimiter{std::move(velocityLimiter)},
    velocityLimiterAtEnd{std::move(velocityLimiterAtEnd)},
    deviationLimiter{std::move(deviationLimiter)}
{
}

CartesianSpaceLinearPath::Iterator::Iterator(const CartesianSpaceLinearPath* parent) :
    PathObjectInterface::IteratorInterface{parent},
    parent{parent},
    t{},
    stepSize{},
    bendItem{}
{
    init();
}

CartesianSpaceLinearPath::Iterator::Iterator(const Iterator& in) :
    PathObjectInterface::IteratorInterface{in},
    parent{in.parent},
    t{in.t},
    stepSize{in.stepSize},
    bendItem{in.bendItem}
{
}

void CartesianSpaceLinearPath::Iterator::init()
{
    const EigenVectord6&  a = parent->startPos.c;
    const EigenVectord6&  b = parent->endPos.c;

    const EigenVectord6 v = (b - a);
    const EigenVectord6 angleScalingVec = EigenVectord6{1.0, 1.0, 1.0,
                s5TranslationLength * 5.0, s5TranslationLength * 5.0, s5TranslationLength * 5.0};
    const double vNormAS = (angleScalingVec.asDiagonal() * v).norm(); 

    stepSize = 0.0004 / vNormAS;

    size_t nrOfSteps = static_cast<size_t>(ceil(1.0 / stepSize));
    stepSize = 1.0 / nrOfSteps;

    requestedVelAtEnd = parent->velocityLimiterAtEnd.getLimit();
}

PathObjectInterface::BendItem CartesianSpaceLinearPath::Iterator::getBendItem()
{
    return bendItem;
}

void CartesianSpaceLinearPath::Iterator::stepImp()
{
    if (t == 1.0)
    {
        PathObjectInterface::IteratorInterface::setEnd();
        return;
    }

    t += stepSize;

    if (t >= 1.0 - stepSize * 0.5)
    {
        t = 1.0;
    }

    updateCurrentBendItem();
}

void CartesianSpaceLinearPath::Iterator::updateCurrentBendItem()
{
    const EigenVectord6&  a = parent->startPos.c;
    const EigenVectord6&  b = parent->endPos.c;

    const EigenVectord6 v = (b - a);
    const double vNorm = v.norm(); 
    CartesianCoordinate current{a + t * v};
    const double dx = stepSize * vNorm;
    CartesianCoordinate smalStep{current.c + v * dx / vNorm};

    JointSpaceCoordinate currentJoint{current};
    JointSpaceCoordinate smalStepJoint{smalStep};

    EigenVectord6 jointSpaceDiff = (smalStepJoint.c - currentJoint.c) / dx;

    double scaleChangeKoefficient = jointSpaceDiff.norm();

    bendItem.pos = currentJoint.c;
    bendItem.requestedVelForLink = parent->velocityLimiter.getLimit(
            std::min(t, 1.0 - t) * vNorm) * scaleChangeKoefficient;

    if (t != 1.0)
    {
        bendItem.requestedVelAtBend = bendItem.requestedVelForLink;
    }
    else
    {
        bendItem.requestedVelAtBend = requestedVelAtEnd * scaleChangeKoefficient;
    }

    bendItem.deviationLimiter = parent->deviationLimiter;
}


std::unique_ptr<PathObjectInterface::IteratorInterface> CartesianSpaceLinearPath::Iterator::makeCopy()
{
    return std::make_unique<Iterator>(*this);
}

PathObjectInterface::Iterator CartesianSpaceLinearPath::begin() const
{
    auto it = std::make_unique<Iterator>(this);
    it->updateCurrentBendItem();
    it->stepImp();

    auto out = std::unique_ptr<PathObjectInterface::IteratorInterface>(std::move(it));

    return PathObjectInterface::Iterator(out);
}

PathObjectInterface::Iterator CartesianSpaceLinearPath::end() const
{
    auto it = std::make_unique<Iterator>(this);
    it->setEnd();
    auto out = std::unique_ptr<PathObjectInterface::IteratorInterface>(std::move(it));

    return PathObjectInterface::Iterator(out);
}

void CartesianSpaceLinearPath::setStart(const EigenVectord6& pos)
{
    startPos = JointSpaceCoordinate(pos);

    const EigenVectord6 v = (endPos.c - startPos.c);

    velocityLimiter.setMoveDir(v);
    velocityLimiterAtEnd.setMoveDir(v);
}
