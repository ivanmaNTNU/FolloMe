#include <SFML/Graphics.hpp>
#include <iostream>
#include "Map.h"
//#include "PathFinder.h"




int main() {
	sf::RenderWindow window(sf::VideoMode(X_PIXELS, Y_PIXELS), "Dynamic Path Finder");


	PathFinder app;

	
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
			app.destination.x = static_cast<int>(ceil(mousePos.x/TILE_PIXEL_SIZE));
			app.destination.y = NUM_Y_TILES - static_cast<int>(ceil(( (Y_PIXELS - mousePos.y)/TILE_PIXEL_SIZE )));
			std::cout << app.destination.x << "   " << app.destination.y << std::endl;
		}

	app.findPath();
	//app.map.update();
	window.clear();
    //window.draw(tile);
	app.map.draw(window);
	window.display();
	}

	

	return 0;
}

