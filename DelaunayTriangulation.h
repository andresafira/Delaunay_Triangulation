//
// Created by andre on 26/12/24.
//

#ifndef DELAUNAYTRIANGULATION_DELAUNAYTRIANGULATION_H
#define DELAUNAYTRIANGULATION_DELAUNAYTRIANGULATION_H

#include <vector>
#include <array>
#include <set>
#include <cfloat>
#include <cmath>

class DelaunayTriangulation{
private:
    std::vector<std::array<float, 2>> points;

    float xMin;
    float xMax;
    float yMin;
    float yMax;

    std::vector<int> badTriangles;
    std::set<std::array<int, 2>> edgeSet;

    std::vector<std::array<int, 3>> triangles;

    bool inCircle(int triangleID, int pointID);

    void getEdgeSet();

    bool edgeFound(std::array<int, 2>& edge, int avoidTriangleID);

public:
    explicit DelaunayTriangulation(std::vector<std::array<float, 2>>& points);

    void update(std::vector<std::array<float, 2>>& points);

    void getTriangulationEdges(std::set<std::array<int, 2>>& edgeSet);

    void getTriangulationTriangles(std::vector<std::array<int, 3>>& triangles);

    static float determinant(float m[3][3]);
};



#endif //DELAUNAYTRIANGULATION_DELAUNAYTRIANGULATION_H
