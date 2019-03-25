#pragma once
#include "lidar.h"
#include <cmath>
#include <SFML/Graphics.hpp>
#include <stack>
#include <vector>
#include <array>
#include <iostream>
//#include "PathFinder.h"

//Size and pixel screen parameters
const double TILE_SIZE = 0.1; //Grid Size in meters
const double X_MAP_SIZE = 20; // Map size along x in meters
const double Y_MAP_SIZE = 20; // Map size along y in meters
const int NUM_X_TILES = floor(X_MAP_SIZE / static_cast<double>(TILE_SIZE));
const int NUM_Y_TILES = floor(Y_MAP_SIZE / static_cast<double>(TILE_SIZE));
const int TILE_PIXEL_SIZE = 4; //Pixels per tile. Each Tile is  square
const int X_PIXELS = TILE_PIXEL_SIZE * NUM_X_TILES; //Window Opened Resolution
const int Y_PIXELS = TILE_PIXEL_SIZE * NUM_Y_TILES;

const double MARGIN_RADIUS = 0.35; //maximal distance in m the robot can get close to a OBSTICLE in its path finding
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
	void drawSafe(sf::RenderWindow& window);
	int getValue(const int& x, const int& y);
	bool setValue(const int& x, const int& y, const int& value);
	int getValueSafe(const int& x, const int& y);
	bool setValueSafe(const int& x, const int& y, const int& value);
	double distance(int x1, int y1, int x2, int y2);
	void discretize(const double& x, const double& y, int& xGrid, int& yGrid);
	bool validate(const int& x, const int& y);
	bool isClearPath(const double& distance);
	void clear();
	friend class PathFinder;

private:
	int** grid = new int*[NUM_X_TILES];
	int** safeGrid = new int*[NUM_X_TILES]; //This is the map that is used with A*. However, it is never seen.
	sf::RectangleShape freeTile;     //all tiles are initialized in the constucter to their preffered colors 
	sf::RectangleShape obsticleTile;
	sf::RectangleShape locationTile;
	sf::RectangleShape destinationTile;
	sf::RectangleShape pathTile;
	int xPos; //NOTE: This is the position of the robot in the array. Not on the map. This is the x = 0; y = 0; on the map. 
	int yPos;
	double angle; // This value comes from the compass and updates in the update() function. 

	Scan scan;
	

};










//--------------------------------------------------------------------------------




const float BIG = 1E10; 
using namespace std;

struct Node
{
	int y;
	int x;
	int parentX;
	int parentY;
	float gCost;
	float hCost;
	float fCost;
};

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
	return lhs.fCost < rhs.fCost;
}


class PathFinder {
public:
	PathFinder(){
		setDestination(0,0);
		foundPath = false;
		//Initialize the variables at the bottom of this document
		//Initialize destination to robot position.
	}
	void setDestination(int x, int y){
		x += map.xPos;
		y += map.yPos;
		destination.fCost = BIG;
		destination.gCost = BIG;
		destination.hCost = BIG;
		destination.parentX = -1;
		destination.parentY = -1;
		destination.x = x;
		destination.y = y;
		//Initialize destination here.
	}

	bool isValid(int x, int y) { //If our Node is an obstacle it is not valid
		if (map.getValueSafe(x - map.xPos,y - map.yPos) != OBSTICLE) { 
			if (x >= floor(NUM_X_TILES) -1 || x <= 0 || y >= floor(NUM_Y_TILES) -1 || y <= 0 ) {
				return false;
			}
			return true;
		}
		return false;
	}

	bool isDestination(int x, int y, Node dest) {
		if (x == dest.x && y == dest.y) {
			return true;
		}
		return false;
	}

	double calculateH(int x, int y, Node dest) {
		double H = (sqrt((x - dest.x)*(x - dest.x)
			+ (y - dest.y)*(y - dest.y)));
		return H;
	}

	void makePath(array<array<Node, NUM_Y_TILES>, NUM_X_TILES> allMap, Node dest) {
		try {
			cout << "Found a path" << endl;
			int x = dest.x;
			int y = dest.y;
			stack<Node> path;


			while (!(allMap[x][y].parentX == x && allMap[x][y].parentY == y)
				&& allMap[x][y].x != -1 && allMap[x][y].y != -1)
			{
				path.push(allMap[x][y]);
				int tempX = allMap[x][y].parentX;
				int tempY = allMap[x][y].parentY;
				x = tempX;
				y = tempY;

			}
			path.push(allMap[x][y]);

			while (!path.empty()) {
				Node top = path.top();
				path.pop();
				//cout << top.x << " " << top.y << endl;
				suggestedPath.emplace_back(top);
				map.setValue(top.x -map.xPos, top.y - map.yPos, PATH);
				//cout << "Path Value set" << endl;
			}
			map.setValue(destination.x - map.xPos, destination.y - map.yPos, DESTINATION);
		}
		catch (const exception& e) {
			cout << e.what() << endl;
		}
	}									//FIX THIS!!!!!!!!!!!!!! RETURNED VALUE SHOULD BE CENTERED AT ZERO.


	void findPath(){ 
		map.update();
		if (isValid(destination.x, destination.y) == false) {
			cout << "Destination is an obstacle" << endl;
			std::cout << destination.x << "   " << destination.y << std::endl;
			foundPath = false;
			return;
			//Destination is invalid
		}
		if (isDestination(map.xPos, map.yPos, destination)) {
			cout << "You are the destination" << endl;
			foundPath = false;
			return;
			//You clicked on yourself
		}
		
		bool closedList[(NUM_X_TILES)][(NUM_Y_TILES)];
		//destination.y = NUM_Y_TILES - destination.y;
		//Initialize whole map
		//Node allMap[50][25];
		array<array<Node, NUM_Y_TILES>, NUM_X_TILES> allMap;
		for (int x = 0; x < NUM_X_TILES; x++) {
			for (int y = 0; y < NUM_Y_TILES; y++) {
				allMap[x][y].fCost = BIG;
				allMap[x][y].gCost = BIG;
				allMap[x][y].hCost = BIG;
				allMap[x][y].parentX = -1;
				allMap[x][y].parentY = -1;
				allMap[x][y].x = x;
				allMap[x][y].y = y;

				closedList[x][y] = false;
			}
		}

		//Initialize our starting list
		int x = map.xPos;
		int y = map.yPos;
		allMap[x][y].fCost = 0.0;
		allMap[x][y].gCost = 0.0;
		allMap[x][y].hCost = 0.0;
		allMap[x][y].parentX = x;
		allMap[x][y].parentY = y;

		vector<Node> openList;
		openList.emplace_back(allMap[x][y]);
		bool destinationFound = false;
		int count = 0;

		while (!openList.empty() && openList.size()<(NUM_X_TILES)*(NUM_Y_TILES)) {
			Node node;
			do {
				//This do-while loop could be replaced with extracting the first
				//element from a set, but you'd have to make the openList a set.
				//To be completely honest, I don't remember the reason why I do
				//it with a vector, but for now it's still an option, although
				//not as good as a set performance wise.
				float temp = BIG;
				vector<Node>::iterator itNode;
				for (vector<Node>::iterator it = openList.begin();
					it != openList.end(); it = next(it)) {
					Node n = *it;
					if (n.fCost < temp) {
						temp = n.fCost;
						itNode = it;
					}
				}
				node = *itNode;
				openList.erase(itNode);
			} while (isValid(node.x, node.y) == false);

			x = node.x;
			y = node.y;
			closedList[x][y] = true;

			//For each neighbour starting from North-West to South-East
			for (int newX = -1; newX <= 1; newX++) {
				for (int newY = -1; newY <= 1; newY++) {
					double gNew, hNew, fNew;
					if (isValid(x + newX, y + newY)) {
						if (isDestination(x + newX, y + newY, destination))
						{
							//Destination found - make path
							allMap[x + newX][y + newY].parentX = x;
							allMap[x + newX][y + newY].parentY = y;
							destinationFound = true;
							makePath(allMap, destination);
							return;
						}
						else if (closedList[x + newX][y + newY] == false)
						{
							if (newX * newY == 0){
								gNew = node.gCost + 1.0;
							}
							else{
								gNew = node.gCost + 1.414;
							}
							
							hNew = calculateH(x + newX, y + newY, destination);
							fNew = gNew + hNew;
							// Check if this path is better than the one already present
							if (allMap[x + newX][y + newY].fCost == BIG ||
								allMap[x + newX][y + newY].fCost > fNew)
							{
								// Update the details of this neighbour node
								allMap[x + newX][y + newY].fCost = fNew;
								allMap[x + newX][y + newY].gCost = gNew;
								allMap[x + newX][y + newY].hCost = hNew;
								allMap[x + newX][y + newY].parentX = x;
								allMap[x + newX][y + newY].parentY = y;
								openList.emplace_back(allMap[x + newX][y + newY]);
							}
						}
					}
				}
			}
			count++;
			if (count >= NUM_X_TILES*NUM_Y_TILES){ //This is so that is something wrong happens, the program does not go into a infinite loop and crash. 
				cout << "Too Heavy" << endl;
				return;
			} 
		}
		if (destinationFound == false) {
			cout << "Destination not found" << endl;
			foundPath = false;
			return;
		}
	}
	
	Map map;
	//PathMap = pathmap; //This is not a object class. This essentially an special map for path finding. The values here come from the map class. This array consists of Nodes.
	Node destination;
	vector<Node> suggestedPath;
	bool foundPath;
	
};





