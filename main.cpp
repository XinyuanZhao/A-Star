#include "AStar.h"

int main() {
	Grid grid;
	grid.enableDiagMove(true);
	grid.draw();

	Pos start{ 6, 6 }, goal{ 14,8 };
	AStar aStar(grid, start, goal);
	aStar.solve();
	aStar.drawSeq();
	aStar.drawPath();
	return 0;
}