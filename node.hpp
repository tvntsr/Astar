/*
    Copyright 2018 Volodymyr Tarasenko

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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

