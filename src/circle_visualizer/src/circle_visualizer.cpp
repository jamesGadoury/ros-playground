#include <SFML/Graphics.hpp>

#include <chrono>
#include <iostream>
#include <random>
#include <functional>
#include <thread>

using namespace std;

struct Node {
    sf::Color color = sf::Color::Blue;
    float x = 0.f;
    float y = 0.f;
    float radius = 0.f;
};

sf::CircleShape convert(const Node &node) {
    sf::CircleShape shape(node.radius);
    shape.setFillColor(node.color);
    shape.setPosition(node.x, node.y);
    return shape;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "wtf" << endl;
        return -1;
    }

    const float width = atof(argv[1]);
    const float height = atof(argv[2]);

    sf::RenderWindow window(
        sf::VideoMode(width, height),
        "SFML FML."
     );

    while (window.isOpen()) {
        auto start = chrono::steady_clock::now();
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        static constexpr float BUFFER = 100.f;
        static constexpr float RADIUS = 100.f;
        static constexpr float OFFSET = BUFFER + RADIUS;

        for (float x = 0.f; x < window.getSize().x - RADIUS; x += OFFSET) {
            for (float y = 0.0f; y < window.getSize().y - RADIUS; y += OFFSET) {
                window.draw(convert(Node{
                    invoke([]() {
                        static mt19937 gen(42);
                        static uniform_real_distribution<float> dis(0.f, 1.f);
                        const float probability = dis(gen);
                        if (probability < 0.1f) {
                            return sf::Color::Red;
                        } else if (probability < 0.6f) {
                            return sf::Color::Green;
                        } else {
                            return sf::Color::Blue;
                        }
                    }),
                    x,
                    y,
                    RADIUS
                }));
            }
        }

        window.display();

        static constexpr auto MINIMUM_UPDATE_DUR = chrono::milliseconds(200);
        if (
            auto update_dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start);
            update_dur < MINIMUM_UPDATE_DUR
        ) {
            this_thread::sleep_for(MINIMUM_UPDATE_DUR - update_dur);
        }
    }

    return 0;
}
