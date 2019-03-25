#pragma once
#include "Map.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include "lidar.h"

//#include "PathFinder.h"

Map::Map()
{
	xPos = floor((float)NUM_X_TILES / 2.0);
	yPos = floor((float)NUM_Y_TILES / 2.0);
	angle = compass.getDirection();
	 
	clear();
	float tileSize = static_cast<float>(TILE_PIXEL_SIZE);
	freeTile.setSize(sf::Vector2f(tileSize, tileSize));
	freeTile.setFillColor(sf::Color::Green);


	obsticleTile.setSize(sf::Vector2f(tileSize, tileSize));
	obsticleTile.setFillColor(sf::Color::Black);


	locationTile.setSize(sf::Vector2f(tileSize, tileSize));
	locationTile.setFillColor(sf::Color::Red);

	
	destinationTile.setSize(sf::Vector2f(tileSize, tileSize));
	destinationTile.setFillColor(sf::Color::White);


	pathTile.setSize(sf::Vector2f(tileSize, tileSize));
	pathTile.setFillColor(sf::Color::Blue);


}
void Map::clear(){
	angle = 0; 

	for (int i = 0; i < NUM_X_TILES; i++) {
		grid[i] = new int[NUM_Y_TILES];
	}
	for (int x = 0; x < NUM_X_TILES; x++) {
		for (int y = 0; y < NUM_X_TILES; y++) {
			grid[x][y] = FREE;
			if (x == xPos && y == yPos) {
				grid[x][y] = LOCATION;
			}
			else
				grid[x][y] = FREE;
		}
	}

	for (int i = 0; i < NUM_X_TILES; i++) {
		safeGrid[i] = new int[NUM_Y_TILES];
	}
	for (int x = 0; x < NUM_X_TILES; x++) {
		for (int y = 0; y < NUM_X_TILES; y++) {
			safeGrid[x][y] = FREE;
			if (x == xPos && y == yPos) {
				safeGrid[x][y] = LOCATION;
			}
			else
				safeGrid[x][y] = FREE;
		}
	}
}



void Map::draw(sf::RenderWindow& window) {
	window.clear(sf::Color::Green);
	for (int x = 0; x < NUM_X_TILES; x++) {
		for (int y = 0; y < NUM_X_TILES; y++) {
			int value = grid[x][y];
			
			if (value == OBSTICLE) {
				obsticleTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(obsticleTile);
			}
			else if (value == PATH) {
				pathTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(pathTile);
			}
			else if (value == LOCATION) {
				locationTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(locationTile);
			}
			else if (value == DESTINATION) {
				destinationTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(destinationTile);
			}
			
		}
	}
}

void Map::drawSafe(sf::RenderWindow& window) {
	window.clear(sf::Color::Green);
	for (int x = 0; x < NUM_X_TILES; x++) {
		for (int y = 0; y < NUM_X_TILES; y++) {
			int value = safeGrid[x][y];
			
			if (value == OBSTICLE) {
				obsticleTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(obsticleTile);
			}
			else if (value == LOCATION) {
				locationTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(locationTile);
			}
			else if (value == DESTINATION) {
				destinationTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(destinationTile);
			}
			else if (value == PATH) {
				pathTile.setPosition(sf::Vector2f(static_cast<float>(x*TILE_PIXEL_SIZE), static_cast<float>(y*TILE_PIXEL_SIZE)));
				window.draw(pathTile);
			}
		}
	}
}


Map::~Map()
{
	for (int i = 0; i < NUM_X_TILES; i++) {
		delete[] grid[i];
	}
	delete grid;
	grid = nullptr;

	for (int i = 0; i < NUM_X_TILES; i++) {
		delete[] safeGrid[i];
	}
	delete safeGrid;
	safeGrid = nullptr;
	scan.stopScan();
}

void Map::update() {
	clear();
	//Update angle fist, then get lidar data. Translate lidar data with respect to angle and same in map.
	scan.update();
	angle = compass.getDirection(); //You can also ajust angle to rotate map to your liking. 
	//get angle from compass.!!!!!!!!!!!!!!!!!!!!!!!
	int xTemp = 0; //this is for tiles in loop
	int yTemp = 0;
	double x = 0; // this is for measurement iteration in loop
	double y = 0;
	//adjust datapoints along compass angle
	for (int dataPoint = 0; dataPoint < scan.numData; dataPoint++) {
		scan.angl[dataPoint] -= 180;//angle;
		if (scan.angl[dataPoint] < 0) {
			scan.angl[dataPoint] += 360;
		}
		if (scan.dist[dataPoint] == 0.0 && scan.quality[dataPoint] == 0.0) {
			continue;
		}
		//std::cout << scan.dist[dataPoint] << "  " << scan.angl[dataPoint] << std::endl;
		x = -scan.dist[dataPoint]/1000 * sin(3.1415/180.0*scan.angl[dataPoint]);
		y = scan.dist[dataPoint] /1000* cos(3.1415/180.0*scan.angl[dataPoint]);
		discretize(x, y, xTemp, yTemp);
		//std::cout << x << "  " << y << std::endl;
		setValue(xTemp, yTemp, OBSTICLE);
		setValueSafe(xTemp, yTemp, OBSTICLE);
	}


}


int Map::getValueSafe(const int& x, const int& y) {
	if (validate(x, y)) {
		return safeGrid[xPos + x][yPos + y];
	}
	else
		return -1;
}
bool Map::setValueSafe(const int& x, const int& y, const int& value) {
	if (validate(x, y)) {
		safeGrid[xPos + x][yPos + y] = value;
		if (value == OBSTICLE){
			for (int itX = x-MARGIN_RADIUS / TILE_SIZE - 1; itX <= x + MARGIN_RADIUS / TILE_SIZE + 1; itX++) {
				for (int itY = y - MARGIN_RADIUS / TILE_SIZE - 1; itY <= y + MARGIN_RADIUS / TILE_SIZE + 1; itY++) {
					if (!validate(itX, itY)){
						continue;
					}
						
					if (distance(x, y, itX, itY) <= MARGIN_RADIUS) { 
						safeGrid[xPos + itX][yPos + itY] = OBSTICLE;
					}
				}
			}
		}
		return true;
	}
	else
		return false;
}

bool Map::setValue(const int& x, const int& y, const int& value) {
	if (validate(x, y)) {
		if (x == 0 && y == 0){	
		}
		else {
			grid[xPos + x][yPos + y] = value;
			return true;
		}
	}
	else
		return false;
}


Scan::Scan() {
	drv = startLidarExpress(); // CAN ALSO USE startLidar(); instead of Express. Fewer points on normal.

	for (int i = 0; i < RPLIDAR_MAX_DATA; i++) {
		dist[i] = 0;
		angl[i] = 0;
		quality[i] = 0; 
	}
	numData = 0;
	//
}

Scan::~Scan() {
	stopLidar(drv);
}

void Scan::stopScan() {
	stopLidar(drv);
}

void Scan::update() {
	numData = lidarData(dist, angl, quality, drv);
	//for (int pos = 0; pos < numData ; ++pos) {
            //printf("theta: %03.2f Dist: %08.2f Q: %d\n", angl[pos], dist[pos], quality[pos]);
            //std::cout << dist[pos] << "  " << angl[pos] << std::endl;
	//get values from rpLidar
	//}
}

bool Map::validate(const int& x, const int& y) {
	if (x >= floor(NUM_X_TILES / 2.0) || x <= -ceil(NUM_X_TILES / 2.0) || y >= floor(NUM_Y_TILES / 2.0) || y <= - ceil(NUM_Y_TILES / 2.0) ){
		return false;
	}
	else
		return true;
}


double Map::distance(int x1, int y1, int x2, int y2) {
	return sqrt(pow( (x1-x2)* TILE_SIZE, 2) + pow( (y1-y2)* TILE_SIZE, 2)); //this is distance given in tiles and returned in meters
}

int Map::getValue(const int& x, const int& y) {
	if (validate(x, y)) {
		return grid[xPos + x][yPos + y];
	}
	else
		return -1;
}




void Map::discretize(const double& x, const double& y, int& xGrid, int& yGrid) {
	xGrid = static_cast<int>(floor(x / TILE_SIZE));
	yGrid = static_cast<int>(floor(y / TILE_SIZE));
}

bool Map::isClearPath(const double& distance) {
	update();
	for (int data = 0; data < scan.numData; data++) {
		if (scan.angl[data] > 360 - atan(ROBOT_WIDTH / LIDAR_DISTANCE_FROM_FRONT / 2.0f) && scan.angl[data] < 360 - atan(ROBOT_WIDTH / ((LIDAR_DISTANCE_FROM_FRONT + distance)* 2.0f))) {
			if (scan.dist[data] <= ROBOT_WIDTH / (2.0f * sin((360 - scan.angl[data]) * 3.1415 / 180.0)))
				return false;
		}
		else if (scan.angl[data] > 360 - atan(ROBOT_WIDTH / ((LIDAR_DISTANCE_FROM_FRONT + distance)* 2.0f)) && scan.angl[data] < 360 ) {
			if (scan.dist[data] < distance)
				return false;
		}
		else if (scan.angl[data] > 0 && scan.angl[data] < atan(ROBOT_WIDTH / ((LIDAR_DISTANCE_FROM_FRONT + distance)* 2.0f))) {
			if (scan.dist[data] < distance)
				return false;
		}
		else if (scan.angl[data] > atan(ROBOT_WIDTH / LIDAR_DISTANCE_FROM_FRONT / 2.0f) && scan.angl[data] < atan(ROBOT_WIDTH / ((LIDAR_DISTANCE_FROM_FRONT + distance)* 2.0f))) {
			if (scan.dist[data] <= ROBOT_WIDTH / (2.0f * sin(scan.angl[data])))
				return false;
		}
	}
	return true;
}
