#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <FolloMe.h>







void FolloMe::screen_run() {
	sf::RenderWindow window(sf::VideoMode(X_PIXELS, Y_PIXELS), "Dynamic Path Finder");
	std::cout << "Started" << std::endl;
	
	Blynk_GPS GPS;
    
    
    
    
    /*while (true){
        std::cin.get();
        GPS.BlynkUpdate();
        std::cout << "--------------------------------------------------------------------------------------------\n";
        std::cout << "Lat: " << GPS.getBlynkLat() << std::endl;

        std::cout << "Lon: " << GPS.getBlynkLon() << std::endl;
        std::cout << "--------------------------------------------------------------------------------------------\n";
        std::cout << std::endl;
    }*/

	
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
			int destinationX = static_cast<int>(ceil(mousePos.x/TILE_PIXEL_SIZE)) - pathFinder.getXPos();
			int destinationY = NUM_Y_TILES - static_cast<int>(ceil(( (Y_PIXELS - mousePos.y)/TILE_PIXEL_SIZE ))) - pathFinder.getYPos();
			pathFinder.setDestination(destinationX, destinationY);
			std::cout << pathFinder.getDestinationX() << "   " << pathFinder.getDestinationY() << std::endl;
		}
		switch (mode){
		case 0:
		
			break;
		case 1:
		
			break;
		case 2:
		
			break;
		case 3:
		
			break;
		default:
		
			break;
		}
		
		std::cout << "Getting GPS" << std::endl;

		std::cout << "Success" << std::endl;
		/*double mapTargetAngle = target.angle + compass.getDirection();
		while (mapTargetAngle < 0){
			mapTargetAngle += 360;
		}
		while (mapTargetAngle >= 360){
			mapTargetAngle -= 360;
		}
		double mapTargetX = target.distance * cos(mapTargetAngle);
		double mapTargetY = target.distance * sin(mapTargetAngle);
		int xGrid;
		int yGrid;
		pathFinder.discretize(mapTargetX, mapTargetY, xGrid, yGrid);
		pathFinder.setDestination(xGrid, yGrid);*/
		
		
		
		pathFinder.findPath();
		
		std::cout << pathFinder.getVectorAngle(3) << std::endl;
		std::cout << "Compass: " << compass.getDirection() << std::endl;

		
		window.clear();
		pathFinder.draw(window);
		window.display();
	}

	

}


int main(){
	FolloMe folloMe;
	std::cout << "Duh Hello" << std::endl;
	folloMe.screen_run();
	
	
	return 0;
}

