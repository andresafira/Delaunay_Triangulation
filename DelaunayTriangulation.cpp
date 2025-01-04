//
// Created by andre on 26/12/24.
//

#include "DelaunayTriangulation.h"

bool xnor(bool a, bool b) {
    return a && b || !(a || b);
}

DelaunayTriangulation::DelaunayTriangulation(std::vector<std::array<float, 2>>& points) {
    this->update(points);
}

void DelaunayTriangulation::update(std::vector<std::array<float, 2>>& points) {
    this->points = points;
    this->badTriangles.clear();
    this->edgeSet.clear();
    this->triangles.clear();

    this->xMax = this->xMin = points[0][0];
    this->yMax = this->yMin = points[0][1];
    float x, y;
    for (auto point : points) {
        x = point[0];
        y = point[1];
        if (this->xMax < x)
            this->xMax = x;
        if (this->xMin > x)
            this->xMin = x;
        if (this->yMax < y)
            this->yMax = y;
        if (this->yMin > y)
            this->yMin = y;
    }

    int nPoints = this->points.size();
    this->points.push_back(std::array<float, 2>{this->xMin - 1, this->yMin - 1});
    this->points.push_back(std::array<float, 2>{this->xMin - 1, 2*this->yMax - this->yMin + 2});
    this->points.push_back(std::array<float, 2>{2*this->xMax - this->xMin + 2, this->yMin - 1});


    this->triangles.push_back(std::array<int, 3>{nPoints, nPoints+1, nPoints+2});

    // Main Loop

    for (int p = 0; p < nPoints; ++p) {
        this->badTriangles.clear();
        this->edgeSet.clear();

        // get triangles which circumcircle contains the point
        for (int t = 0; t < this->triangles.size(); ++t) {
            if (this->inCircle(t, p)) {
                this->badTriangles.push_back(t);
            }
        }

        this->getEdgeSet();
        // remove such triangles
        for (int t = this->badTriangles.size() - 1; t >= 0; --t){
            this->triangles.erase(this->triangles.begin() + this->badTriangles[t]);
        }
        // add new triangles from unique edges
        for (auto edge : this->edgeSet) {
            this->triangles.push_back(std::array<int, 3>{edge[0], edge[1], p});
        }
    }

    // remove super triangles
    this->badTriangles.clear();
    for (int t = 0; t < this->triangles.size(); ++t) {
        int t0 = this->triangles[t][0];
        int t1 = this->triangles[t][1];
        int t2 = this->triangles[t][2];

        if (t0 >= nPoints || t1 >= nPoints || t2 >= nPoints) {
            this->badTriangles.push_back(t);
        }
    }

    for (int t = this->badTriangles.size() - 1; t >= 0; --t)
        this->triangles.erase(this->triangles.begin() + this->badTriangles[t]);

    this->badTriangles.clear();
    this->edgeSet.clear();
}

bool DelaunayTriangulation::inCircle(int triangleID, int pointID) {
    auto t = this->triangles[triangleID];
    auto a = this->points[t[0]];
    auto b = this->points[t[1]];
    auto c = this->points[t[2]];
    auto d = this->points[pointID];

    float ax = a[0];
    float ay = a[1];
    float bx = b[0];
    float by = b[1];
    float cx = c[0];
    float cy = c[1];
    float dx = d[0];
    float dy = d[1];

    bool isCounterclockwise = (bx - ax)*(cy - by) > (by - ay)*(cx - bx);

    float matrix[3][3] = {{ax-dx, ay-dy, (ax-dx)*(ax-dx) + (ay-dy)*(ay-dy)},
                           {bx-dx, by-dy, (bx-dx)*(bx-dx) + (by-dy)*(by-dy)},
                           {cx-dx, cy-dy, (cx-dx)*(cx-dx) + (cy-dy)*(cy-dy)}};

    return xnor(isCounterclockwise, DelaunayTriangulation::determinant(matrix) >= 0);
}

float DelaunayTriangulation::determinant(float m [3][3]) {
    float ans = m[0][0]*m[1][1]*m[2][2] + m[0][1]*m[1][2]*m[2][0] + m[0][2]*m[1][0]*m[2][1];
    ans -= m[2][0]*m[1][1]*m[0][2] + m[2][1]*m[1][2]*m[0][0] + m[2][2]*m[1][0]*m[0][1];
    return ans;
}

bool edgeEqual(std::array<int, 2>& a, std::array<int, 2>& b){
    bool first = (a[0] == b[0]) && (a[1] == b[1]);
    bool second = (a[1] == b[0]) && (a[0] == b[1]);
    return first || second;
}

bool DelaunayTriangulation::edgeFound(std::array<int, 2>& edge, int avoidTriangleID) {
    for (int t : this->badTriangles) {
        if (t == avoidTriangleID)
            continue;

        std::vector<int> triangle = {this->triangles[t][0],
                                     this->triangles[t][1],
                                     this->triangles[t][2]};
        for (int i = 0; i < 3; ++i) {
            std::array<int, 2> newEdge = {triangle[i], triangle[(i+1)%3]};
            if (edgeEqual(edge, newEdge))
                return true;
        }
    }

    return false;
}

void DelaunayTriangulation::getEdgeSet() {
    for (int t : this->badTriangles){
        std::vector<int> triangle = {this->triangles[t][0],
                                     this->triangles[t][1],
                                     this->triangles[t][2]};
        for (int i = 0; i < 3; ++i) {
            std::array<int, 2> edge = {triangle[i], triangle[(i+1)%3]};
            if (!this->edgeFound(edge, t)) {
                this->edgeSet.insert(edge);
            }
        }
    }
}

void  DelaunayTriangulation::getTriangulationEdges(std::set<std::array<int, 2>>& edgeSet) {
  edgeSet.clear();

  for (auto t : this->triangles) {
    int triangle[3] = {t[0], t[1], t[2]};
    for (int i = 0; i < 3; ++i) {
      std::array<int, 2> edge{};
      if (triangle[i] < triangle[(i+1)%3]) {
        edge = {triangle[i], triangle[(i+1)%3]};
      } else {
        edge = {triangle[(i+1)%3], triangle[i]};
      }
      if (!edgeSet.contains(edge)) {
        edgeSet.insert(edge);
      }
    }
  }
}
void DelaunayTriangulation::getTriangulationTriangles(std::vector<std::array<int, 3>>& triangles) {
    triangles = this->triangles;
}
