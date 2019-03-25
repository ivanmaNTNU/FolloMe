
//#pragma once
#include <stack>
#include <vector>
#include <array>
#include <iostream>
#include <SFML/Graphics.hpp>
#include "Map.h"
#ifndef PATHFINDER_H
#define PATHFINDER_H
//typedef array<array<Node, NUM_Y_TILES>, NUM_X_TILES> PathMap;
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

	static bool isValid(int x, int y) { //If our Node is an obstacle it is not valid
		if (map.getValueSafe(x,y) != OBSTICLE)) { 
			if (x >= floor(NUM_X_TILES / 2.0) || x <= -ceil(NUM_X_TILES / 2.0) || y >= floor(NUM_Y_TILES / 2.0) || y <= - ceil(NUM_Y_TILES / 2.0) ) {
				return false;
			}
			return true;
		}
		return false;
	}

	static bool isDestination(int x, int y, Node dest) {
		if (x == dest.x && y == dest.y) {
			return true;
		}
		return false;
	}

	static double calculateH(int x, int y, Node dest) {
		double H = (sqrt((x - dest.x)*(x - dest.x)
			+ (y - dest.y)*(y - dest.y)));
		return H;
	}

	static void makePath(array<array<Node, NUM_Y_TILES>, NUM_X_TILES> map, Node dest) {
		try {
			cout << "Found a path" << endl;
			int x = dest.x;
			int y = dest.y;
			stack<Node> path;


			while (!(map[x][y].parentX == x && map[x][y].parentY == y)
				&& map[x][y].x != -1 && map[x][y].y != -1)
			{
				path.push(map[x][y]);
				int tempX = map[x][y].parentX;
				int tempY = map[x][y].parentY;
				x = tempX;
				y = tempY;

			}
			path.push(map[x][y]);

			while (!path.empty()) {
				Node top = path.top();
				path.pop();
				//cout << top.x << " " << top.y << endl;
				suggestedPath.emplace_back(top);
				map.setValue(top.x, top.y, PATH);
			}
		}
		catch (const exception& e) {
			cout << e.what() << endl;
		}
	}									//FIX THIS!!!!!!!!!!!!!! RETURNED VALUE SHOULD BE CENTERED AT ZERO.


	static void findPath(){ 
		vector<Node> empty;
		if (isValid(destination.x, destination.y) == false) {
			cout << "Destination is an obstacle" << endl;
			return empty;
			//Destination is invalid
		}
		if (isDestination(map.xPos, map.yPos, destination)) {
			cout << "You are the destination" << endl;
			return empty;
			//You clicked on yourself
		}
		bool closedList[(NUM_X_TILES)][(NUM_Y_TILES)];

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
						}
						else if (closedList[x + newX][y + newY] == false)
						{
							gNew = node.gCost + 1.0;
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
		}
		if (destinationFound == false) {
			cout << "Destination not found" << endl;
			foundPath = false;
		}
	}
	
	Map map;
	//PathMap = pathmap; //This is not a object class. This essentially an special map for path finding. The values here come from the map class. This array consists of Nodes.
	Node destination;
	vector<Node> suggestedPath;
	bool foundPath;
	
};
#endif
