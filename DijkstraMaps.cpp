// Based on the Console template
// This is quick and dirty code.


#include <list>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <vector>
#include <DirectXMath.h>
#include <Windows.h>


constexpr int W_WALL = INT16_MAX;

using MapMatrix = std::vector<std::vector<int>>;
using namespace DirectX;

class PathfindingDMap	// Dijkstra map
{
public:
	PathfindingDMap() {
		neighbours[0] = XMINT2(0, -1); neighbours[1] = XMINT2(-1, 0);
		neighbours[2] = XMINT2(0, 1); neighbours[3] = XMINT2(1, 0);
	}

	void reset() {
		m_map = nullptr;
		m_dmap = nullptr;
	}

	int FillDMap(MapMatrix* _goalsDMap, MapMatrix* _stepweightsMap)
	{
		m_map = _stepweightsMap;
		m_dmap = _goalsDMap;
		XMINT2 neighbour;

		int _mapw = _stepweightsMap->at(0).size();
		int _maph = _stepweightsMap->size();

		// Start by filling the filledDMap with the starting DMap
		m_filledDMap = MapMatrix(*m_dmap);

		// next keep calculating until the map doesn't change any more
		bool isStable = false;
		int ctIterations = 0;
		while (!isStable)
		{
			isStable = true;
			for (int i = 0; i < _mapw; i++)
			{
				for (int j = 0; j < _mapw; j++)
				{
					if (m_filledDMap[i][j] == W_WALL)		// wall
						continue;
					for (auto n : neighbours) {
						neighbour.x = i + n.x;
						neighbour.y = j + n.y;
						// wrap around or not
						if (neighbour.x < 0)
							neighbour.x = _mapw - 1;
							// continue;
						if (neighbour.x == _mapw)	// w
							neighbour.x = 0;
							// continue;
						if (neighbour.y < 0)
							// neighbour.y = _maph - 1;
							continue;
						if (neighbour.y == _maph)	// h
							// neighbour.y = 0;
							continue;

						auto neighbourVal = m_filledDMap[neighbour.x][neighbour.y];
						// check if the neighbour is lower value (including movement cost)
						auto stepval = (*m_map)[i][j] + 1;
						if ((m_filledDMap[i][j] - neighbourVal) > stepval)
						{
							// set the value to the neighbour + 1, plus the movement cost
							m_filledDMap[i][j] = min(neighbourVal + stepval, W_WALL - 1);
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
	XMINT2 neighbours[4];
};

int main(int argc, char* argv[]) {
	srand(clock());
	MapMatrix tmp_matrix_steps = {	// step costs, in [j][i]
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

	// Transpose the matrix so that we do operations against [x][y]

	int _mapw = tmp_matrix_steps.size();
	int _maph = tmp_matrix_steps.at(0).size();

	MapMatrix matrix_steps = MapMatrix(tmp_matrix_steps);
	MapMatrix matrix_basemap = MapMatrix(matrix_steps);
	int dmapVal;
	for (int i = 0; i < _mapw; i++)
	{
		for (int j = 0; j < _mapw; j++)
		{
			// transpose and calculate
			matrix_steps[i][j] = tmp_matrix_steps[j][i];
			switch (matrix_steps[i][j])
			{
			case 9:		// wall
				dmapVal = W_WALL;
				break;
			case 1:		// door
			case 4:		// firewall
			default:	// normal
				dmapVal = W_WALL - 1;
				break;
			}
			matrix_basemap[i][j] = dmapVal;
		}
	};

	PathfindingDMap dm;
	
	matrix_basemap[28][9] = 0;	// Set a goal;
	matrix_basemap[15][23] = 0;	// Set a goal;
	matrix_basemap[10][5] = 0;	// Set a goal;

	clock_t start;
	clock_t diff;
	int msec = 0;
	start = clock();
	int iters = 1000;

	for (int a = 0; a < iters; a++)
	{
		dm.FillDMap(&matrix_basemap, &matrix_steps);
		dm.reset();
	}

	diff = clock() - start;
	msec = diff * 1000 / CLOCKS_PER_SEC;
	printf("Time taken for %d Dijkstra Maps: %d seconds %d milliseconds\n", iters, msec / 1000, msec % 1000);

	auto ctIter = dm.FillDMap(&matrix_basemap, &matrix_steps);
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
	for (int y = -1; y <= _maph; y++) {
		// for (int x = -1; x <= MAPW; x++) {	// to add vertical walls
		for (int x = 0; x < _mapw; x++) {		// no vertical walls
			if (x < 0 || y < 0 || x > (_mapw - 1) || y > (_maph - 1) || matrix_basemap[x][y] == W_WALL)
				std::cout << char(0xdb) << char(0xdb) << char(0xdb) << char(0xdb);	// Draw walls
			else if ((*filledDMap)[x][y] > 0)
			{
				char buf[30];
				sprintf_s(buf, "%4d", (*filledDMap)[x][y]);
				if (matrix_steps[x][y] == 1)	// door
					std::cout << "\033[1;44m" << buf << "\033[0m";
				else if (matrix_steps[x][y] == 4)	// fire wall
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