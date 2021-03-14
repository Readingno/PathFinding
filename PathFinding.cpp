
#include <iostream>
using namespace std;

#include "olcConsoleGameEngine.h"


class PathFinding : public olcConsoleGameEngine
{
public:

	PathFinding()
	{
		m_sAppName = L"Path Finding";
	}

private:

	struct Node
	{
		int x;					// Position of the node
		int y;
		bool isWall;			// Whether this node is a wall
		bool isVisited;			// Have we already searched this node
		float traveledDis;		// How long have been traveled
		float goalDis;			// Heuristic distance to the goal
		vector<Node*> neighbours;	// Neighbour nodes
		Node* parent;			// Parent node, where did it come from
	};

	Node *nodes = nullptr;		// All the nodes in the field
	Node *nStart = nullptr;		// Starting node
	Node *nEnd = nullptr;		// End node

	int gridWidth = 32;
	int gridHeight = 32;

protected:

	virtual bool OnUserCreate() 
	{
		// Initialize the grid
		nodes = new Node[gridWidth * gridHeight];
		for (int x = 0; x < gridWidth; x++) 
		{
			for (int y = 0; y < gridHeight; y++) 
			{
				nodes[y * gridWidth + x].x = x;
				nodes[y * gridWidth + x].y = y;
				nodes[y * gridWidth + x].isWall = false;
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}

		// Get all the neighbours for nodes
		for (int x = 0; x < gridWidth; x++) 
		{
			for (int y = 0; y < gridHeight; y++) 
			{
				if (y > 0)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y - 1) * gridWidth + x]);
				if (y < gridHeight - 1)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y + 1) * gridWidth + x]);
				if (x > 0)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[y * gridWidth + x - 1]);
				if (x < gridWidth - 1)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[y * gridWidth + x + 1]);
				/* Can walk diagonally*/
				if (y > 0 && x > 0)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y - 1) * gridWidth + x - 1]);
				if (y < gridHeight - 1 && x > 0)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y + 1) * gridWidth + x + 1]);
				if (y > 0 && x < gridWidth - 1)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y - 1) * gridWidth + x + 1]);
				if (y < gridHeight - 1 && x < gridWidth - 1)
					nodes[y * gridWidth + x].neighbours.push_back(&nodes[(y + 1) * gridWidth + x + 1]);
				//*/
			}
		}

		// Create default start and end nodes
		nStart = &nodes[(gridHeight / 2) * gridWidth + 13];
		nEnd = &nodes[(gridHeight / 2 + 1) * gridWidth - 14];

		return true;
	}

	virtual bool OnUserUpdate(float deltaTime)
	{
		int gridSize = 5;
		int borderSize = 1;
		
		// Get the node selected by mouse
		int selectedNodeX = m_mousePosX / gridSize;
		int selectedNodeY = m_mousePosY / gridSize;

		// Get mouse press
		if (m_mouse[0].bPressed)
		{
			// Left mouse —— change wall
			if (selectedNodeX >= 0 && selectedNodeX < gridWidth && selectedNodeY >= 0 && selectedNodeY < gridHeight)
			{
				nodes[selectedNodeY * gridWidth + selectedNodeX].isWall = !nodes[selectedNodeY * gridWidth + selectedNodeX].isWall;
				//DFSPathFinding();
				//BFSPathFinding();
				//GreedyBFSPathFinding();
				//Dijkstra();
				AStarPathFinding();
			}
		}
		if (m_mouse[1].bPressed)
		{
			if (selectedNodeX >= 0 && selectedNodeX < gridWidth && selectedNodeY >= 0 && selectedNodeY < gridHeight)
			{
				// Right mouse and Shift —— change end node
				if (m_keys[VK_SHIFT].bHeld)
				{
					nEnd = &nodes[selectedNodeY * gridWidth + selectedNodeX];
					//DFSPathFinding();
					//BFSPathFinding();
					//GreedyBFSPathFinding();
					//Dijkstra();
					AStarPathFinding();
				}
				// Right mouse —— change start node
				else
				{
					nStart = &nodes[selectedNodeY * gridWidth + selectedNodeX];
					//DFSPathFinding();
					//BFSPathFinding();
					//GreedyBFSPathFinding();
					//Dijkstra();
					AStarPathFinding();
				}
			}
		}

		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		// Draw connection
		for (int x = 0; x < gridWidth; x++)
		{
			for (int y = 0; y < gridHeight; y++)
			{
				if (!nodes[y * gridWidth + x].isWall)
				{
					for (auto n : nodes[y * gridWidth + x].neighbours)
					{
						if (!n->isWall)
						{
							DrawLine(x * gridSize + gridSize / 2, y * gridSize + gridSize / 2,
								n->x * gridSize + gridSize / 2, n->y * gridSize + gridSize / 2, PIXEL_HALF, FG_GREY);
						}
					}
				}
			}
		}

		// Draw grid
		for (int x = 0; x < gridWidth; x++) 
		{
			for (int y = 0; y < gridHeight; y++) 
			{
				// Draw gorund and wall
				Fill(x * gridSize + borderSize, y * gridSize + borderSize,
					(x + 1) * gridSize - borderSize, (y + 1) * gridSize - borderSize, 
					PIXEL_HALF, nodes[y * gridWidth + x].isWall? FG_BLACK: FG_GREY);
				//Draw visited nodes
				if (nodes[y * gridWidth + x].isVisited) 
				{
					Fill(x * gridSize + borderSize, y * gridSize + borderSize,
						(x + 1) * gridSize - borderSize, (y + 1) * gridSize - borderSize,
						PIXEL_SOLID, FG_DARK_GREY);
				}
				// Draw start node
				if (&nodes[y * gridWidth + x] == nStart)
				{
					Fill(x * gridSize + borderSize, y * gridSize + borderSize,
						(x + 1) * gridSize - borderSize, (y + 1) * gridSize - borderSize,
						PIXEL_SOLID, FG_GREEN);
				}
				//Draw end node
				if (&nodes[y * gridWidth + x] == nEnd)
				{
					Fill(x * gridSize + borderSize, y * gridSize + borderSize,
						(x + 1) * gridSize - borderSize, (y + 1) * gridSize - borderSize,
						PIXEL_SOLID, FG_RED);
				}
			}
		}

		// Draw path
		Node* p = nEnd;
		while (p->parent != nullptr)
		{
			DrawLine(p->x * gridSize + gridSize / 2, p->y * gridSize + gridSize / 2,
				p->parent->x * gridSize + gridSize / 2, p->parent->y * gridSize + gridSize / 2,
				PIXEL_SOLID, FG_YELLOW);
			p = p->parent;
		}

		return true;
	}

	void AStarPathFinding() 
	{
		// Initialize all the nodes
		for (int x = 0; x < gridWidth; x++) 
		{
			for (int y = 0; y < gridWidth; y++) 
			{
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].goalDis = Distance(&nodes[y * gridWidth + x], nEnd);
				nodes[y * gridWidth + x].traveledDis = INFINITY;
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}

		Node* nCurrent = nullptr;
		list<Node*> fringe;
		nStart->traveledDis = 0.0f;
		fringe.push_back(nStart);
		while (!fringe.empty() && nCurrent != nEnd)
		{
			// Get the best next move
			nCurrent = fringe.front();
			for (Node* n : fringe)
			{
				if ((n->traveledDis + n->goalDis) < (nCurrent->traveledDis + nCurrent->goalDis))
				{
					nCurrent = n;
				}
			}
			nCurrent->isVisited = true;
			// Update te fringe list
			for (Node* n : nCurrent->neighbours)
			{
				if (n->isWall || n->isVisited) continue;
				for (Node* neighbour : n->neighbours)
				{
					if (neighbour->isVisited && neighbour->traveledDis + Distance(n, neighbour) < n->traveledDis)
					{
						n->traveledDis = neighbour->traveledDis + Distance(n, neighbour);
						n->parent = neighbour;
					}
				}
				fringe.push_back(n);
			}
			fringe.remove(nCurrent);
		}
	}

	void GreedyBFSPathFinding()
	{
		// Initialize all the nodes
		for (int x = 0; x < gridWidth; x++)
		{
			for (int y = 0; y < gridWidth; y++)
			{
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].goalDis = Distance(&nodes[y * gridWidth + x], nEnd);
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}

		Node* nCurrent = nullptr;
		list<Node*> fringe;
		fringe.push_back(nStart);
		while (!fringe.empty() && nCurrent != nEnd)
		{
			// Get the best next move
			nCurrent = fringe.front();
			for (Node* n : fringe)
			{
				if (n->goalDis < nCurrent->goalDis)
				{
					nCurrent = n;
				}
			}
			nCurrent->isVisited = true;
			// Update te fringe list
			for (Node* n : nCurrent->neighbours)
			{
				if (n->isWall || n->isVisited) continue;
				for (Node* neighbour : n->neighbours)
				{
					float minDis = INFINITY;
					if (neighbour->isVisited && neighbour->goalDis < minDis)
					{
						n->parent = neighbour;
						minDis = neighbour->goalDis;
					}
				}
				fringe.push_back(n);
			}
			fringe.remove(nCurrent);
		}
	}

	void Dijkstra()
	{
		// Initialize all the nodes
		for (int x = 0; x < gridWidth; x++)
		{
			for (int y = 0; y < gridWidth; y++)
			{
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].traveledDis = INFINITY;
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}

		Node* nCurrent = nullptr;
		list<Node*> fringe;
		nStart->traveledDis = 0.0f;
		fringe.push_back(nStart);
		while (!fringe.empty() && nCurrent != nEnd)
		{
			// Get the best next move
			nCurrent = fringe.front();
			for (Node* n : fringe)
			{
				if (n->traveledDis < nCurrent->traveledDis)
				{
					nCurrent = n;
				}
			}
			nCurrent->isVisited = true;
			// Update te fringe list
			for (Node* n : nCurrent->neighbours)
			{
				if (n->isWall || n->isVisited) continue;
				for (Node* neighbour : n->neighbours)
				{
					if (neighbour->isVisited && neighbour->traveledDis + Distance(n, neighbour) < n->traveledDis)
					{
						n->traveledDis = neighbour->traveledDis + Distance(n, neighbour);
						n->parent = neighbour;
					}
				}
				fringe.push_back(n);
			}
			fringe.remove(nCurrent);
		}
	}

	void BFSPathFinding()
	{
		// Initialize all the nodes
		for (int x = 0; x < gridWidth; x++)
		{
			for (int y = 0; y < gridWidth; y++)
			{
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}
		Node* nCurrent = nullptr;
		list<Node*> fringe;
		fringe.push_back(nStart);
		while (!fringe.empty() && nCurrent != nEnd)
		{
			// Get the best next move
			nCurrent = fringe.front();
			fringe.pop_front();
			nCurrent->isVisited = true;
			// Update te fringe list
			for (Node* n : nCurrent->neighbours)
			{
				if (n->isWall || n->isVisited) continue;
				n->parent = nCurrent;
				fringe.push_back(n);
			}
		}
	}

	void DFSPathFinding()
	{
		// Initialize all the nodes
		for (int x = 0; x < gridWidth; x++)
		{
			for (int y = 0; y < gridWidth; y++)
			{
				nodes[y * gridWidth + x].isVisited = false;
				nodes[y * gridWidth + x].parent = nullptr;
			}
		}
		Node* nCurrent = nullptr;
		list<Node*> fringe;
		fringe.push_back(nStart);
		while (!fringe.empty() && nCurrent != nEnd)
		{
			// Get the best next move
			nCurrent = fringe.front();
			fringe.pop_front();
			nCurrent->isVisited = true;
			// Update te fringe list
			for (Node* n : nCurrent->neighbours)
			{
				if (n->isWall || n->isVisited) continue;
				n->parent = nCurrent;
				fringe.push_front(n);
			}
		}
	}

	float Distance(Node* a, Node* b) {
		return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
	}
};

int main()
{
	PathFinding game;
	game.ConstructConsole(160, 160, 5, 5);
	game.Start();
	return 0;
}
