////////////////////////////////////////////////////////////
// Headers
////////////////////////////////////////////////////////////
#include <SFML/Graphics.hpp>

#include <cstdlib>

////////////////////////////////////////////////////////////
/// Constants
////////////////////////////////////////////////////////////
#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600

void render_finger()
{
    // Create the window of the application with a stencil buffer
    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}),
                            "SFML Stencil",
                            sf::Style::Titlebar | sf::Style::Close,
                            sf::State::Windowed,
                            sf::ContextSettings{0 /* depthBits */, 8 /* stencilBits */});
    window.setVerticalSyncEnabled(true);


    sf::RectangleShape Base_Link({150, 40});
    Base_Link.setFillColor(sf::Color::Red);
    Base_Link.setPosition({320, 450});
    Base_Link.setRotation(sf::degrees(90));

    sf::RectangleShape Link1({150, 40});
    Link1.setFillColor(sf::Color::Red);
    Link1.setPosition({300, 430});
    Link1.setRotation(sf::degrees(0));

    sf::CircleShape Joint1(30);
    Joint1.setFillColor(sf::Color::Blue);
    Joint1.setPosition({270, 420});
    Joint1.setRotation(sf::degrees(0));

    while (window.isOpen())
    {
        // Handle events
        while (const std::optional event = window.pollEvent())
        {
            // Window closed: exit
            if (event->is<sf::Event::Closed>())
            {
                window.close();
                break;
            }
        }

        // Clear the window color to white and the initial stencil buffer values to 0
        window.clear(sf::Color::White, 0);

        /*
        `* Draw Finger
         * Joints on Level 7-5 and links on level 4-1
        `*/ 

        window.draw(Base_Link,
                    sf::StencilMode{sf::StencilComparison::Always, sf::StencilUpdateOperation::Replace, 4, ~0u, false});

        window.draw(Link1,
                    sf::StencilMode{sf::StencilComparison::Always, sf::StencilUpdateOperation::Replace, 3, ~0u, false});

        window.draw(Joint1,
                    sf::StencilMode{sf::StencilComparison::Greater, sf::StencilUpdateOperation::Replace, 7, ~0u, false});

        // Display things on screen
        window.display();
    }

    // return EXIT_SUCCESS;
}
