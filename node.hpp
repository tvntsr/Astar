#ifndef NODE_DEF_HPP_
#define NODE_DEF_HPP_

#include <math.h>

namespace Node
{

typedef std::pair<int, int> point2d;

struct node
{
    // current position
    point2d pos;

    // total distance already travelled to reach the node
    int distance;

    // total time already spent 
    int time;

    // node priority
    int priority;

public:

    node(const point2d& point)
        : pos(point)
        , distance(0)
        , time(0)
        , priority(0)
    {}

    const point2d& getPos() const
    {
        return pos;
    }

    int getDistance() const
    {
        return distance;
    }

    int getPriority() const
    {
        return priority;
    }

    int getTime() const
    {
        return time;
    }

    void forcePriority(int newPriority)
    {
        priority = newPriority;
    }

    void initialPriority(const point2d & dest)
    {
        priority = estimate(dest) * 100;
    }

    // Estimation function for the remaining distance to the goal.
    const int estimate(const point2d& dest) const
    {
        int xd, yd, d;

        xd = dest.first - pos.first;
        yd = dest.second- pos.second;

        // Euclidian Distance
        d = sqrt(xd*xd + yd*yd);

        // Manhattan distance
        //d=abs(xd)+abs(yd);

        // Chebyshev distance
        //d=max(abs(xd), abs(yd));

        return d;
    }
};

// Determine priority (in the priority queue)
bool operator>(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

bool operator==(const node & a, const node & b)
{
    return (a.getPos().first == b.getPos().first) && (a.getPos().second == b.getPos().second);
}

}; //namespace

#endif //NODE_DEF_HPP_

