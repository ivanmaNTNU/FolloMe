#pragma once
#include "lidar.h"
#include <cmath>
#include <SFML/Graphics.hpp>

//Size and pixel screen parameters
const double TILE_SIZE = 0.3; //Grid Size in meters
const double X_MAP_SIZE = 10; // Map size along x in meters
const double Y_MAP_SIZE = 10; // Map size along y in meters
const int NUM_X_TILES = floor(X_MAP_SIZE / static_cast<double>(TILE_SIZE));
const int NUM_Y_TILES = floor(Y_MAP_SIZE / static_cast<double>(TILE_SIZE));
const int TILE_PIXEL_SIZE = 10; //Pixels per tile. Each Tile is  square
const int X_PIXELS = TILE_PIXEL_SIZE * NUM_X_TILES; //Window Opened Resolution
const int Y_PIXELS = TILE_PIXEL_SIZE * NUM_Y_TILES;

const double MARGIN_RADIUS = 0.3; //maximal distance in m the robot can get close to a OBSTICLE in its path finding
const double ROBOT_WIDTH = 0.8; //Width of robot in meters. 
const double LIDAR_DISTANCE_FROM_FRONT = 0.2; //Where lidar is mounted relative to robot rear in meters. 
const int RPLIDAR_MAX_DATA = 1000; //How large should the arrays be for rdlidar to avoid overflow. 

#define OBSTICLE 1
#define FREE 0
#define LOCATION 2
#define DESTINATION 3
#define PATH 4



class Scan {
public:
	Scan();
	~Scan();
	void update();
	friend class Map;
	void stopScan();

private:
	float dist[RPLIDAR_MAX_DATA];
	float angl[RPLIDAR_MAX_DATA];
	int quality[RPLIDAR_MAX_DATA];
	int numData; //Number of data points gathered in one revolution of lidar scan. 
	rp::standalone::rplidar::RPlidarDriver* drv;
};

class Map
{
public:
	Map();
	~Map();
	void update();
	void draw(sf::RenderWindow& window);
	int getValue(const int& x, const int& y);
	bool setValue(const int& x, const int& y, const int& value);
	int getValueSafe(const int& x, const int& y);
	bool setValueSafe(const int& x, const int& y, const int& value);
	double distance(int x1, int y1, int x2, int y2);
	void discretize(const double& x, const double& y, int& xGrid, int& yGrid);
	bool validate(const int& x, const int& y);
	bool isClearPath(const double& distance);
private:
	int** grid = new int*[NUM_X_TILES];
	int** safeGrid = new int*[NUM_X_TILES]; //This is the map that is used with A*. However, it is never seen.
	//sf::RectangleShape freeTile;     //all tiles are initialized in the constucter to their preffered colors 
	sf::RectangleShape obsticleTile;
	sf::RectangleShape locationTile;
	sf::RectangleShape destinationTile;
	sf::RectangleShape pathTile;
	int xPos; //NOTE: This is the position of the robot in the array. Not on the map. This is the x = 0; y = 0; on the map. 
	int yPos;
	double angle; // This value comes from the compass and updates in the update() function. 

	Scan scan;
	

};





