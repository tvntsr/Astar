# Algorithm

A* algorithm is used to find the shorten path.

At each iteration of its main loop, A* needs to determine which of its paths to extend. It does so based on the cost of the path and an estimate of the cost required to extend the path all the way to the goal. Specifically, A* selects the path that minimizes

```
f(n) = g(n) + h(n)
```
where:

  * n is the next node on the path,
  * g(n) is the cost of the path from the start node to n,
  * h(n) is a heuristic function that estimates the cost of the cheapest path from n to the goal

## g(n)

g(n) calculated as Euclidian distance and implemented in node::estimate method:

```C++
    const int estimate(const point2d& dest) const
    {
        int xd, yd, d;

        xd = dest.first - pos.first;
        yd = dest.second- pos.second;

        // Euclidian Distance
        d = sqrt(xd*xd + yd*yd);

        return d;
    }
```

also g(n) could be calculated as Manhattan distance:
```C
        d = abs(xd) + abs(yd);
```
or as Chebyshev distance:
```C
        d = max(abs(xd), abs(yd));
```

## h(n)

h(h) in this task calculated as time function and the following is used:

  * time depends on the movement direction
  * time is bigger when rover going up
  * time is less when rover going down
  * time depends on the elevation
  * time calculate in miliseconds

# Implementation

A* algorithm is implemented as template function

```C++
template <typename UnitType, typename PathNode, int Directions, typename Queue >
    std::forward_list<std::pair<UnitType, UnitType> > pathFind(const std::pair<UnitType, UnitType> wSize
                     , const std::pair<UnitType, UnitType>& start
                     , const std::pair<UnitType, UnitType>& finish
                     , const std::vector<std::pair<int, int>> directionsArray
                     , std::function<int(PathNode& to, const PathNode& from, const int movement)> priorityCalculation
                     , std::function<bool(const std::pair<UnitType, UnitType>&)> obstacleFilter )
```

## Template parameters

Template parameters are:

  * UnitType - defines type to keep cartesian coordinates, preferable std::pair<>
  * PathNode - defines a node on the map
  * Queue - priority queue used in A* algorithm
  * Directions - integer, defines possible movement in the mpa cell

## Function parameters

Functions paremeters are:

  * wSize - defines world size for calculation, pair<x, y>
  * start - defines the point, the route is searching from, pair<x, y>
  * finish - defines the point, the route is searching to, pair<x, y>
  * directionsArray - vector of all posible movement in the cell, it contains movement as pair <x, y>, vector should contain Directions element
  * priorityCalculation -  function to calulate priority for the node, parameters for this function is current note, parent node and directions index [0..Directions)
  * obstacleFilter - function to check obstacles on the way, parameter for this function is par<x, y>

## Return

Function returns list of points defining shortest path

## Requirements for the PathNode type:

PathNode should provide the following methods:

  * constructor, take pair<x, y>, defining node coordinates on the map
  * copy constructor and assigment operator available, used to store node into the queue
  * getPos() - returns pair<<UnitType, UnitType>, coordinates of the node on the map
  * getPriority() - returns priority for the node
  * initialPriority(const pair<x, y>), sets initial priority for the node according to the passed coordinates

Priority of the node calculated in priorityCalculation fuction.
