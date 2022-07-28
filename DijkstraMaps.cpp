// Based on the Console template
// This is quick and dirty code.


#include <list>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <Windows.h>

// Use our own min/max
#define NOMINMAX

#define  MAX(a,b)          (((a) > (b)) ? (a) : (b))
#define  MIN(a,b)          (((a) < (b)) ? (a) : (b))

#define MAPW 32
#define MAPH 32

using MapMatrix = char[MAPW][MAPH];

class PathPoint {
public:
	PathPoint(int a = 0, int b = 0) { x = a; y = b; }
	bool operator ==(const PathPoint& o) { return o.x == x && o.y == y; }
	PathPoint operator +(const PathPoint& o) {
		return { o.x + x, o.y + y };
	}
	int x, y;
};

class PathfindingDMap	// Dijkstra map
{
public:
	PathfindingDMap() {
		neighbours[0] = PathPoint(0, -1); neighbours[1] = PathPoint(-1, 0);
		neighbours[2] = PathPoint(0, 1); neighbours[3] = PathPoint(1, 0);
	}

	void reset() {
		m_map = nullptr;
		m_dmap = nullptr;
	}

	int FillDMap(MapMatrix* _goalsDMap, MapMatrix* _stepweightsMap)
	{
		m_map = _stepweightsMap;
		m_dmap = _goalsDMap;
		PathPoint neighbour;

		// Start by filling the filledDMap with the starting DMap
		for (int j = 0; j < MAPH; j++)
		{
			for (int i = 0; i < MAPW; i++)
			{
				auto dval = (*m_dmap)[i][j];
				m_filledDMap[i][j] = dval;
			}
		}

		// next keep calculating until the map doesn't change any more
		bool isStable = false;
		int ctIterations = 0;
		while (!isStable)
		{
			isStable = true;
			for (int j = 0; j < MAPH; j++)	// h
			{
				for (int i = 0; i < MAPW; i++)	// w
				{
					if (m_filledDMap[i][j] == 127)		// wall
						continue;
					for (auto n : neighbours) {
						neighbour.x = i + n.x;
						neighbour.y = j + n.y;
						// wrap around or not
						if (neighbour.x < 0)
							neighbour.x = MAPW - 1;
							// continue;
						if (neighbour.x == MAPW)	// w
							neighbour.x = 0;
							// continue;
						if (neighbour.y < 0)
							// neighbour.y = MAPH - 1;
							continue;
						if (neighbour.y == MAPH)	// h
							// neighbour.y = 0;
							continue;

						auto neighbourVal = m_filledDMap[neighbour.x][neighbour.y];
						// check if the neighbour is lower value (including movement cost)
						if ((m_filledDMap[i][j] - neighbourVal) > ((*m_map)[i][j] + 1))
						{
							// set the value to the neighbour + 1, plus the movement cost
							m_filledDMap[i][j] = MIN(neighbourVal + (*m_map)[i][j] + 1, 125);
							isStable = false;
						}
					};
				}
			}
			++ctIterations;
		}

		return ctIterations;
	}

	MapMatrix* GetFinalDMap() { return &m_filledDMap; };

	MapMatrix* m_map = nullptr;
	MapMatrix* m_dmap = nullptr;	// starting dmap

private:
	MapMatrix m_filledDMap;		// filled dmap
	PathPoint neighbours[4];
};

int main(int argc, char* argv[]) {
	srand(clock());
	MapMatrix tmp_m = {	// step costs
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{4, 4, 4, 4, 4, 4, 4, 9, 1, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{4, 4, 4, 4, 4, 4, 4, 9, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 1, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 1, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 1, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 9, 0, 1, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 9, 0, 9, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 9, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 9, 9, 9, 1, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	};
	// Reverse the matrices so it is correct
	// We needed the temp ones to make them humanly readable
	MapMatrix m, dmmap;
	int dmapVal;
	for (int j = 0; j < MAPH; j++)
	{
		for (int i = 0; i < MAPW; i++)
		{
			m[i][j] = tmp_m[j][i];
			switch (m[i][j])
			{
			case 9:		// wall
				dmapVal = 127;
				break;
			case 1:		// door
			case 4:		// firewall
			default:	// normal
				dmapVal = 126;
				break;
			}
			dmmap[i][j] = dmapVal;
		}
	};

	PathfindingDMap dm;
	
	dmmap[28][9] = 0;	// Set a goal;
	dmmap[15][23] = 0;	// Set a goal;
	dmmap[10][5] = 0;	// Set a goal;

	clock_t start;
	clock_t diff;
	int msec = 0;
	start = clock();
	int iters = 1000;

	for (int a = 0; a < iters; a++)
	{
		dm.FillDMap(&dmmap, &m);
		dm.reset();
	}

	diff = clock() - start;
	msec = diff * 1000 / CLOCKS_PER_SEC;
	printf("Time taken for %d Dijkstra Maps: %d seconds %d milliseconds\n", iters, msec / 1000, msec % 1000);

	auto ctIter = dm.FillDMap(&dmmap, &m);
	std::cout << "Stable at iteration " << ctIter << "\n";

	auto filledDMap = dm.GetFinalDMap();

	// Set output mode to handle virtual terminal sequences
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hOut == INVALID_HANDLE_VALUE)
	{
		return GetLastError();
	}
	DWORD dwMode = 0;
	if (!GetConsoleMode(hOut, &dwMode))
	{
		return GetLastError();
	}
	dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
	if (!SetConsoleMode(hOut, dwMode))
	{
		return GetLastError();
	}
	for (int y = -1; y <= MAPH; y++) {
		// for (int x = -1; x <= MAPW; x++) {	// to add vertical walls
		for (int x = 0; x < MAPW; x++) {		// no vertical walls
			if (x < 0 || y < 0 || x > (MAPW - 1) || y > (MAPH - 1) || dmmap[x][y] == 127)
				std::cout << char(0xdb) << char(0xdb) << char(0xdb) << char(0xdb);	// Draw walls
			else if ((*filledDMap)[x][y] > 0)
			{
				char buf[5];
				sprintf_s(buf, "%4d", (*filledDMap)[x][y]);
				if (m[x][y] == 1)	// door
					std::cout << "\033[1;44m" << buf << "\033[0m";
				else if (m[x][y] == 4)	// fire wall
					std::cout << "\033[1;41m" << buf << "\033[0m";
				else
				{
					if ((*filledDMap)[x][y] > 14)
						std::cout << "\033[1;31m" << buf << "\033[0m";
					else if ((*filledDMap)[x][y] > 9)
						std::cout << "\033[1;33m" << buf << "\033[0m";
					else
						std::cout << "\033[1;32m" << buf << "\033[0m";
				}
			}
			else {	// goal
				std::cout << "\033[1;42m" << "GOAL" << "\033[0m";
			}
		}
		std::cout << "\n";
	}

	return 0;
}