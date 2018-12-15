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

#ifndef ASTAR_HPP_
#define ASTART_HPP_

#include <functional>
#include <forward_list>
#include <stdexcept>


namespace astar
{

/// A-star algorithm.
/// Template parameters are:
/// \param UnitType - defines type to keep cartesian coordinates, preferable std::pair<>
/// \param PathNode - defines a node on the map
/// \param Queue - priority queue used in A* algorithm
/// \param Directions - integer, defines possible movement in the mpa cell
///
/// Functions paremeters are:
/// \param wSize - defines world size for calculation, pair<x, y>
/// \param start - defines the point, the route is searching from, pair<x, y>
/// \param finish - defines the point, the route is searching to, pair<x, y>
/// \param directionsArray - vector of all posible movement in the cell, it contains movement as pair <x, y>, vector should contain Directions element
/// \param priorityCalculation -  function to calulate priority for the node, parameters for this function is current note, parent node and directions index [0..Directions)
/// \param obstacleFilter - function to check obstacles on the way, parameter for this function is par<x, y>
///
/// \return returning list of points defining shortest path
///
/// Requirements for the PathNode type:
///  it should provide the following methods:
///   - constructor, take pair<x, y>, defining node coordinates on the map
///   - getPos() - returns pair<<UnitType, UnitType>, coordinates of the node on the map
///   - getPriority() - returns priority for the node
///   - initialPriority(const pair<x, y>), sets initial priority for the node according to the passed coordinates
///  Priority of the node calculated in priorityCalculation fuction.
template <typename UnitType, typename PathNode, int Directions, typename Queue >
    std::forward_list<std::pair<UnitType, UnitType> > pathFind(const std::pair<UnitType, UnitType> wSize
                     , const std::pair<UnitType, UnitType>& start
                     , const std::pair<UnitType, UnitType>& finish
                     , const std::vector<std::pair<int, int>> directionsArray
                     , std::function<int(PathNode& to, const PathNode& from, const int movement)> priorityCalculation
                     , std::function<bool(const std::pair<UnitType, UnitType>&)> obstacleFilter )
{
    struct visiting 
    {
        int closed;
        int open;
        int dir;
    };

    typedef std::pair<UnitType, UnitType> point2d;

    auto world_size = wSize.first * wSize.second + wSize.first;

    if (Directions  != directionsArray.size())
        throw std::range_error("Wrong directionsArray size");

    std::vector<visiting> maps;
    maps.resize(world_size);

    Queue pq; // list of open (not-yet-tried) nodes


    // create the start node and push into list of open nodes
    PathNode n0(start);
    n0.initialPriority(finish); 

    pq.push(n0);
    int x = n0.getPos().first;
    int y = n0.getPos().second;

    int xy_to_pos = wSize.first*y+x;

    maps[xy_to_pos].open = n0.getPriority(); // mark it on the open nodes map

    std::forward_list<std::pair<UnitType, UnitType> > ret;

    // do A* search
    while(!pq.empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        PathNode n0 = pq.top();

        x = n0.getPos().first;
        y = n0.getPos().second;

        xy_to_pos = wSize.first*y + x;

        pq.pop(); // remove the node from the open list

        maps[xy_to_pos].open = 0;

        // mark it on the closed nodes map
        maps[xy_to_pos].closed = 1;

        // quit searching when the goal state is reached
        if(n0.estimate(finish) == 0)
        {
            // generate the path from finish to start
            // by following the directions
            ret.push_front(start);

            while(!(x == start.first && y == start.second))
            {
                xy_to_pos = wSize.first * y + x;

                ret.push_front(std::make_pair(x, y));

                int j = maps[xy_to_pos].dir;

                x += directionsArray[j].first;
                y += directionsArray[j].second;
            }

            return ret;
        }

        // generate moves (child nodes) in all possible directions
        for(int i = 0; i < Directions; ++i)
        {
            point2d point = std::make_pair( x + directionsArray[i].first, y + directionsArray[i].second);

            int point_pos = wSize.first * point.second + point.first;

            if(!(  point.first < 0  || point.first > wSize.first - 1
                || point.second < 0 || point.second > wSize.second - 1
                || obstacleFilter(point) 
                || maps[point_pos].closed == 1))
            {
                // generate a child node
                PathNode m0(point);

                priorityCalculation(m0, n0, i);

                // if it is not in the open list then add into that
                if(maps[point_pos].open == 0)
                {
                    maps[point_pos].open = m0.getPriority();
                    pq.push(m0);
                    // sets the direction to its parent node
                    maps[point_pos].dir = (i + Directions/2) % Directions;
                }
                else if(maps[point_pos].open > m0.getPriority())
                {
                    // update the priority info
                    maps[point_pos].open = m0.getPriority();
                    // update the direction to the parent node
                    maps[point_pos].dir = (i + Directions/2) % Directions;
                    pq.pop();
                    pq.push(m0); // add the better node instead
                }
            }
        }
    }
    return ret; // no route found
};

} // Namespace
#endif //ASTAR_HPP_
