#include <Eigen/Dense>
#include <algorithm>
//#include <limits>
#include <Eigen/StdVector>

#ifndef DUMMYTRAJECTORYGENERATOR_H
#define DUMMYTRAJECTORYGENERATOR_H

template <int N, class NumTyp>
class RobotDynamics
{
  public:
    virtual void update(const Eigen::Matrix<NumTyp, N, 1>& pos, const Eigen::Matrix<NumTyp, N, 1>& preVelDir,
                        const Eigen::Matrix<NumTyp, N, 1>& postVelDir, NumTyp ApproxVel) = 0;

    const Eigen::Matrix<NumTyp, N, N>&
    getA()
    {
        return a;
    }

    const Eigen::Matrix<NumTyp, N, N>&
    getB()
    {
        return b;
    }

    const Eigen::Matrix<NumTyp, N, 1>&
    getTorqueLimits()
    {
        return torqueLimits;
    }

    const NumTyp&
    getDt()
    {
        return dt;
    }

    const Eigen::Matrix<NumTyp, N, N>&
    getBInv()
    {
        return bInv;
    }

    const Eigen::Matrix<NumTyp, N, 1>&
    getExternalTorqueAcc()
    {
        return externalTorqueAcc;
    }

    const Eigen::Matrix<NumTyp, N, 1>&
    getMaxAxisAbsVel()
    {
        return maxAxisAbsVel;
    }

  protected:
    virtual ~RobotDynamics()
    {
    }

  protected:
    Eigen::Matrix<NumTyp, N, N> a;
    Eigen::Matrix<NumTyp, N, N> b;
    Eigen::Matrix<NumTyp, N, 1> torqueLimits;
    NumTyp dt;

    Eigen::Matrix<NumTyp, N, N> bInv;
    Eigen::Matrix<NumTyp, N, 1> externalTorqueAcc;

    Eigen::Matrix<NumTyp, N, 1> maxAxisAbsVel;

    template<int N1, class NumTyp1>
    friend std::ostream& operator<<(std::ostream& os, const RobotDynamics<N1, NumTyp1>& dt);
};

template <int N, class NumTyp>
std::ostream& operator<<(std::ostream& os, const RobotDynamics<N, NumTyp>& d)
{
    os << "a = \n" << d.a << "\n";
    os << "b = \n" << d.b << "\n";
    os << "torqueLimits = \n" << d.torqueLimits << "\n";
    os << "bInv = \n" << d.bInv << "\n";
    os << "externalTorqueAcc = \n" << d.externalTorqueAcc << "\n";
    return os;
}

template<int N, class NumTyp>
class TrajectoryItem
{
  public:
    Eigen::Matrix<NumTyp, N, 1> p;
    Eigen::Matrix<NumTyp, N, 1> v;
    Eigen::Matrix<NumTyp, N, 1> u;
};

template<int N, class NumTyp>
TrajectoryItem<N, NumTyp> operator*(const TrajectoryItem<N, NumTyp>& a, float t)
{
    TrajectoryItem<N, NumTyp> out{a};
    out.p *= t;
    out.v *= t;
    out.u *= t;
    return out;
}

template<int N, class NumTyp>
TrajectoryItem<N, NumTyp> operator*(float t, const TrajectoryItem<N, NumTyp>& a)
{
    return a * t;
}

template<int N, class NumTyp>
TrajectoryItem<N, NumTyp> operator+(const TrajectoryItem<N, NumTyp>& a, const TrajectoryItem<N, NumTyp>& b)
{
    TrajectoryItem<N, NumTyp> out{a};
    out.p += b.p;
    out.v += b.v;
    out.u += b.u;
    return out;
}

template<int N, class NumTyp>
TrajectoryItem<N, NumTyp> operator-(const TrajectoryItem<N, NumTyp>& a, const TrajectoryItem<N, NumTyp>& b)
{
    TrajectoryItem<N, NumTyp> out{a};
    out.p -= b.p;
    out.v -= b.v;
    out.u -= b.u;
    return out;
}

template<int N, class NumTyp>
TrajectoryItem<N, NumTyp> operator/(const TrajectoryItem<N, NumTyp>& a, float t)
{
    TrajectoryItem<N, NumTyp> out{a};
    out.p /= t;
    out.v /= t;
    out.u /= t;
    return out;
}

// This is just a simple implementation to get started
class DummyTrajectoryGenerator
{
private:
    class BendPoint
    {
    public:
        Eigen::Matrix<double, 6, 1> pos;
        double velocity;
    };

public:
    DummyTrajectoryGenerator(RobotDynamics<6, double>* dynamics, double stub)
    {
        changeRobotDynamics(dynamics);
    }

    virtual ~DummyTrajectoryGenerator() {}
    void changeRobotDynamics(RobotDynamics<6, double>* dynamics)
    {
        this->dynamics = dynamics;
    }

    void setStart(const Eigen::Matrix<double, 6, 1>& pos, double velocity, double stub = std::numeric_limits<double>::max())
    {
        BendPoint bendPoint{pos, velocity};

        if (bendPoints.size() == 0)
        {
            bendPoints.push_back(std::move(bendPoint));
        }
        else
        {
            bendPoints[0] = bendPoint;
        }
    }

    void addBendPoint(const Eigen::Matrix<double, 6, 1>& pos, double stub1, double velocity,
                      double stub2 = std::numeric_limits<double>::max())
    {
        bendPoints.push_back(BendPoint{pos, velocity});
    }

    void clear()
    {
        bendPoints.clear();
    }

    void calculateTrajectory()
    {
        trajectoryItems.clear();

        double lastVel = bendPoints[0].velocity;
        Eigen::Matrix<double, 6, 1> lastPos = bendPoints[0].pos;
        Eigen::Matrix<double, 6, 1> firstDir = bendPoints[1].pos - lastPos;
        firstDir.normalize();

        auto item = TrajectoryItem<6, double>{lastPos, lastVel * firstDir, Eigen::Matrix<double, 6, 1>::Zero()};
        trajectoryItems.push_back(item);

        for (auto it = ++std::cbegin(bendPoints); it != std::cend(bendPoints); ++it)
        {
            const auto& current = *it;
            double currentVel = current.velocity;
            insertTrajItems(lastPos, current.pos, lastVel, currentVel);
            lastPos = current.pos;
            lastVel = currentVel;
        }
    }

    auto begin() const
    {
        return std::cbegin(trajectoryItems);
    }

    auto end() const
    {
        return std::cend(trajectoryItems);
    }

private:
    RobotDynamics<6, double>* dynamics;
    std::vector<BendPoint> bendPoints;
    std::vector<TrajectoryItem<6, double> > trajectoryItems;

    void insertTrajItems(const Eigen::Matrix<double, 6, 1>& pos0, const Eigen::Matrix<double, 6, 1>& pos1,
            double v0, double& v1)
    {
        const double& dt = dynamics->getDt();
        const Eigen::Matrix<double, 6, 1>& maxAxisAbsVel = dynamics->getMaxAxisAbsVel();

        Eigen::Matrix<double, 6, 1> dirVec = pos1 - pos0;
        double l = dirVec.norm();
        dirVec /= l;

        // fix for v0 == 0 or v1 == 0 resulting in inf loop
        // v0 == v1 == 0 will still result in inf loop
        {
            double approxT = (l / ((v0 + v1) / 2.0));
            // minVel is approximately the velocity one timestep ahead of v0 if const acc
            // between v0 and v1 was applied (or one timestep behind v1)
            double minVel = std::min(v0, v1) + abs(v1 - v0) * dt / approxT;
            v0 = std::max(minVel, v0);
            v1 = std::max(minVel, v1);
        }

        auto dirIt = std::cbegin(dirVec);
        auto maxAxisVelIt = std::cbegin(maxAxisAbsVel);

        double maxAxisVel = std::numeric_limits<double>::max();
        for (; dirIt != std::cend(dirVec) && maxAxisVelIt != std::cend(maxAxisAbsVel); ++dirIt, ++maxAxisVelIt)
        {
            maxAxisVel = std::min(maxAxisVel, *maxAxisVelIt / std::abs(*dirIt));
        }

        v0 = std::min(v0, maxAxisVel);
        v1 = std::min(v1, maxAxisVel);

        // l = integral(v0 + c * p(t) dt, from 0 to T)
        // v(t) = dp/dt(t) = v0 + c * p(t), if c != 0 => 
        //  | p(t) = -v0 / c + a * exp(c * t)
        //  | v(t) = a * c * exp(c * t)
        // p(T) = l
        // v(0) = v0
        // v(T) = v1 = v0 + c * p(T) =>
        //  | c = (v1 - v0) / l

        double c = (v1 - v0) / l;

        double moveT;
        std::function<double(double)> p;
        std::function<double(double)> v;

        if (abs(c) > std::numeric_limits<double>::min() * 10000)
        {
            // if v1 != v0 => c != 0 =>
            // p(0) = 0 = -v0 / c + a * exp(0) =>
            //  | a = v0 / c
            // ==>
            // p(t) = v0 / c * (exp(c* t) - 1)
            // v(t) = v0 * exp(c * t)
            moveT = log(l * c / v0 + 1) / c;
            p = [v0, c](double t) {return v0 / c * (exp(c * t) - 1);};
            v = [v0, c](double t) {return v0 * exp(c * t);};
        }
        else
        {
            // if v1 == v0 => c == 0 =>
            // l = integral(v0 dt, from 0 to T)
            // v(t) = v0 => 
            //  | p(t) = v0 * t
            moveT = l / v0;
            p = [v0, c](double t) {return v0 * t;};
            v = [v0, c](double t) {return v0;};
        }

        size_t timesteps = ceil(moveT / dt);
        double moveScaling = l / p(timesteps * dt);
        dirVec *= moveScaling;

        for (double t = dt; t < (timesteps + 0.5) * dt; t += dt)
        {
            Eigen::Matrix<double, 6, 1> posN = pos0 + p(t) * dirVec;
            const Eigen::Matrix<double, 6, 1>& posNm1 = trajectoryItems.back().p;
            v1 = v(t);
            auto item = TrajectoryItem<6, double>{posN,
                    (posN - posNm1) / dt,
                    //recalculated later on so just putting zeros for u
                    Eigen::Matrix<double, 6, 1>::Zero()};
            trajectoryItems.push_back(item);
        }
    }
};

#endif /* DUMMYTRAJECTORYGENERATOR_H */
