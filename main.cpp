#include <iostream>
#include <vector>
#include <array>
#include <random>
#include "DelaunayTriangulation.h"
#include <SFML/Graphics.hpp>

int colorClip(int color) {
    if (color > 255)
        return 255;
    if (color < 0)
        return 0;
    return color;
}

void drawPoints(std::vector<std::array<float, 2>>& points,
                sf::RenderWindow& window) {
    for (int i = 0; i < points.size(); ++i) {
        sf::CircleShape shape(5.f);
        shape.setPosition({points[i][0] - shape.getRadius(), points[i][1] - shape.getRadius()});
        shape.setFillColor(sf::Color::Red);
        window.draw(shape);
    }
}

void drawEdges(std::set<std::array<int, 2>>& edges, std::vector<std::array<float, 2>>& points,
               sf::RenderWindow& window) {
    for (auto edge : edges) {
        std::array line =
        {
            sf::Vertex{sf::Vector2f(points[edge[0]][0],
                                    points[edge[0]][1]), sf::Color::Black},
            sf::Vertex{sf::Vector2f(points[edge[1]][0],
                                    points[edge[1]][1]), sf::Color::Black}
        };

        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
    }
}

void drawTriangles(std::vector<std::array<int, 3>>& triangles, std::vector<std::array<float, 2>>& points,
               std::vector<sf::Color>& pointColors, sf::RenderWindow& window) {
    for (auto t : triangles) {
        sf::VertexArray triangle(sf::PrimitiveType::Triangles, 3);

        // define the position of the triangle's points
        float mean_x, mean_y;
        mean_x = 0;
        mean_y = 0;
        for (int j = 0; j < 3; ++j) {
            triangle[j].position = sf::Vector2f(points[t[j]][0], points[t[j]][1]);
            mean_x += points[t[j]][0] / 3;
            mean_y += points[t[j]][1] / 3; 
        }
        int colorIndex = mean_x / 1100 * 255;
        int greenIndex = mean_y / 600 * 255;
        
        colorIndex = colorClip(colorIndex);
        greenIndex = colorClip(greenIndex);

        for (int j = 0; j < 3; ++j) {
            triangle[j].color = sf::Color(colorIndex, greenIndex, 255 - colorIndex);
        }

        window.draw(triangle);
    }
}

void updatePoints(std::vector<std::array<float, 2>>& points,
                  std::vector<std::array<float, 2>>& velocity,
                  std::vector<sf::Color>& pointColors) {
    for (int i = 0; i < points.size(); ++i) {
        points[i][0] += velocity[i][0];
        points[i][1] += velocity[i][1];

        if (points[i][0] < 5) {
            points[i][0] = 5;
            velocity[i][0] *= -1;
        } else if (points[i][0] > 1095) {
            points[i][0] = 1095;
            velocity[i][0] *= -1;
        }

        if (points[i][1] < 5) {
            points[i][1] = 5;
            velocity[i][1] *= -1;
        } else if (points[i][1] > 595) {
            points[i][1] = 595;
            velocity[i][1] *= -1;
        }
        int colorIndex = points[i][1]/600*255;
        int greenIndex = points[i][0]/1100*255;

        colorIndex = colorClip(colorIndex);
        greenIndex = colorClip(greenIndex);

        pointColors[i] = sf::Color(colorIndex, greenIndex, 255 - colorIndex);
    } 
}


void initPoints(std::vector<std::array<float, 2>>& points,
                std::vector<std::array<float, 2>>& velocity,
                std::vector<sf::Color>& pointColors, int size) {
    float posXLow = 5;
    float posXUpper = 1095;
    std::uniform_real_distribution<float> posXDist(posXLow, posXUpper);
    
    float posYLow = 5;
    float posYUpper = 595;
    std::uniform_real_distribution<float> posYDist(posYLow, posYUpper);
    
    float vLow = -0.04;
    float vUpper = 0.04;
    std::uniform_real_distribution<float> vDist(vLow, vUpper);

    std::default_random_engine re;

    points.clear();
    pointColors.clear();
    velocity.clear();
    points.resize(size);
    pointColors.resize(size);
    velocity.resize(size);

    for (int i = 0; i < size; ++i) {
        points[i] = std::array<float, 2>{posXDist(re), posYDist(re)};
        int colorIndex = points[i][1]/600*255;
        pointColors[i] = sf::Color(colorIndex, 0, 255 - colorIndex);
        velocity[i] = std::array<float, 2>{vDist(re), vDist(re)};
    }
    points.push_back(std::array<float, 2>{0, 0});
    points.push_back(std::array<float, 2>{0, 600});
    points.push_back(std::array<float, 2>{1100, 600});
    points.push_back(std::array<float, 2>{1100, 0});
    
    
    pointColors.push_back(sf::Color::Blue);
    pointColors.push_back(sf::Color::Red);
    pointColors.push_back(sf::Color::Red);
    pointColors.push_back(sf::Color::Blue);

    velocity.push_back(std::array<float, 2>{0, 0});
    velocity.push_back(std::array<float, 2>{0, 0});
    velocity.push_back(std::array<float, 2>{0, 0});
    velocity.push_back(std::array<float, 2>{0, 0});
     
}


int main() {
    std::vector<std::array<float, 2>> points;
    std::vector<std::array<float, 2>> velocity;
    std::vector<sf::Color> pointColors;

    initPoints(points, velocity, pointColors, 50);
    
    auto DT = DelaunayTriangulation(points);

    sf::RenderWindow window(sf::VideoMode({1100, 600}), "SFML works!");

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        updatePoints(points, velocity, pointColors);
        
        window.clear();
        std::vector<std::array<int, 3>> triangles;
        std::set<std::array<int, 2>> edges;
        DT.getTriangulationTriangles(triangles);
        DT.getTriangulationEdges(edges);
        drawTriangles(triangles, points, pointColors, window);
        // drawEdges(edges, points, window);
        // drawPoints(points, window);
        window.display();

        DT.update(points);
    }

    return 0;
}
