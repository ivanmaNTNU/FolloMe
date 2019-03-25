#include <SFML/Graphics.hpp>
#include <iostream>
#include "Map.h"
//#include "pathFinder.h"




int main() {
	sf::RenderWindow window(sf::VideoMode(X_PIXELS, Y_PIXELS), "Dynamic Path Finder");
	sf::RectangleShape tile(sf::Vector2f(300.0f, 300.0f));
	tile.setOrigin(sf::Vector2f(150.0f, 150.0f));
	tile.setPosition(sf::Vector2f(600, 450));
	tile.setFillColor(sf::Color::Green);
	tile.setOutlineColor(sf::Color::Red);
	tile.setOutlineThickness(5.0f);

	Map map;
	delay(7000);
	
	while (window.isOpen()) {
		sf::Event evnt;

		while (window.pollEvent(evnt)) {
			switch (evnt.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::TextEntered:
				if (evnt.text.unicode < 128 && evnt.text.unicode != 13) {
					std::cout << static_cast<char>(evnt.text.unicode);
				}
				else if (evnt.text.unicode == 13) {
					std::cout << std::endl;
				}
			}
		}
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			sf::Vector2i mousePos = sf::Mouse::getPosition(window);
			tile.setPosition(sf::Vector2f(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y)));
		}


	map.update();
	window.clear();
    //window.draw(tile);
	map.draw(window);
	window.display();

	}

	

	return 0;
}

