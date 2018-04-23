#include "AStar.h"

vector<Pos> Grid::DIRS = { Pos{1,0}, Pos{-1,0}, Pos{0,1}, Pos{0,-1},
						Pos{1,1}, Pos{1,-1}, Pos{-1,-1}, Pos{-1,1} };

Grid::Grid() {
	height = 15, width = 15;
	roughTerrainCost = 5, diagCost = 1.4;
	dir = 4;
	setObstacle({ 3,5 }, { 6,3 });
	setRoughTerrain({ 9,12 }, { 11,4 });
}

Grid::Grid(int _h, int _w, double _roughTerrainCost, double _diagCost) {
	height = _h, width = _w;
	roughTerrainCost = _roughTerrainCost;
	diagCost = _diagCost;
	dir = 4;
}

void Grid::setObstacle(const Pos _tl, const Pos _br) {
	for (int i = _tl.x; i <= _br.x; ++i)
		for (int j = _br.y; j <= _tl.y; ++j) {
			obstacle.emplace(Pos{ i, j });
		}
}

void Grid::clearObstacle() {
	obstacle.clear();
}

void Grid::setRoughTerrain(const Pos _tl, const Pos _br) {
	for (int i = _tl.x; i <= _br.x; ++i)
		for (int j = _br.y; j <= _tl.y; ++j) {
			roughTerrain.emplace(Pos{ i, j });
		}
}

void Grid::clearRoughTerrain() {
	roughTerrain.clear();
}

void Grid::setRoughTerrainCost(const double _cost) {
	roughTerrainCost = _cost;
}

void Grid::setDiagCost(const double _cost) {
	diagCost = _cost;
}

void Grid::enableDiagMove(const bool _flag) {
	if (_flag) dir = 8;
	else dir = 4;
}

int Grid::getHeight() const {
	return height;
}

int Grid::getWidth() const {
	return width;
}

vector<Pos> Grid::neighbors(const Pos _p) const {
	vector<Pos> result;
	for (int i = 0; i < dir; ++i) {
		Pos p = _p + DIRS[i];
		if (isValid(p)) result.push_back(p);
	}
	return result;
}

vector<vector<char>> Grid::getGrid() const {
	int x, y;
	vector<vector<char>> result(height, vector<char>(width));
	for (int i = 0; i < height; ++i)
		for (int j = 0; j < width; ++j) {
			x = j + 1;
			y = height - i;
			Pos p{ x,y };
			if (obstacle.find(p) != obstacle.end()) result[i][j] = '#';
			else if (roughTerrain.find(p) != roughTerrain.end()) result[i][j] = '*';
			else result[i][j] = '.';
		}
	return result;
}

double Grid::getCost(const Pos _from, const Pos _to) const {
	if (roughTerrain.count(_from) != 0 || roughTerrain.count(_to) != 0)
		return roughTerrainCost;
	return 1.0;
}

bool Grid::isValid(const Pos _p) const {
	if (_p.x <= 0 || _p.x > width || _p.y <= 0 || _p.y > height || obstacle.count(_p) != 0)
		return false;
	return true;
}

void Grid::draw() const {
	cout << "\n This is the original grid. \n";
	cout << "\'#\' stands for obstacles. \'*\' stands for rough terrain. "
		<< "\'.\' stands for flat terrain. \n";
	draw(getGrid());
}

void Grid::draw(const vector<vector<char>>& _grid) {
	if (_grid.size() == 0 || _grid[0].size() == 0) return;
	int height = _grid.size(), width = _grid[0].size();
	for (int i = 0; i < height; ++i) {
		cout << endl << "\t";
		for (int j = 0; j < width; ++j) cout << _grid[i][j] << " ";
	}
	cout << endl << endl;
}

AStar::AStar(const Grid _grid, const Pos _start, const Pos _goal) {
	grid = _grid;
	start = _start;
	goal = _goal;
	heuristic = manhattan;
	counts = 0;
	assert(grid.isValid(start) && grid.isValid(goal));
}

void AStar::setHeuristic(const function<double(Pos, Pos)> _H) {
	heuristic = _H;
}

void AStar::solve() {
	vector<vector<char>> curGrid(grid.getHeight(), vector<char>(grid.getWidth(), '.'));
	frontier.push(Node{ start, heuristic(start, goal), counts });
	parent[start] = start;
	cost[start] = heuristic(start, goal);
	while (!frontier.empty()) {
		counts++;
		Node cur = frontier.top();
		frontier.pop();
		updateGrid(curGrid, &cur.position, nullptr);
		if (cur.position == goal) break;
		for (Pos next : grid.neighbors(cur.position)) {
			double new_cost = cost[cur.position] + grid.getCost(cur.position, next);
			if (cost.find(next) == cost.end() || new_cost < cost[next]) {
				cost[next] = new_cost;
				frontier.push(Node{ next, new_cost + heuristic(next, goal), counts });
				parent[next] = cur.position;
				updateGrid(curGrid, nullptr, &next);
			}
		}
		gridSeq.push_back(curGrid);
	}
	gridSeq.push_back(curGrid);

	Pos cur(goal);
	while (cur != start) {
		path.push_back(cur);
		cur = parent[cur];
	}
	path.push_back(cur);
	reverse(path.begin(), path.end());
}

void AStar::drawSeq() const {
	if (gridSeq.size() == 0) {
		cout << "No solutions yet.\n";
		return;
	}
	cout << "\n Here comes the solution sequence of A*.\n";
	cout << "\'o\' stands for points marked as frontiers. \'+\' stands for points visited.\n";
	int count = 0;
	for (auto i : gridSeq) {
		count++;
		cout << "The " << count << " step:" << endl;
		Grid::draw(i);
	}
}

void AStar::drawPath() const {
	if (path.size() == 0) {
		cout << "No solutions yet.\n";
		return;
	}
	cout << "\n Here comes the final path.\n";
	cout << "\'#\' stands for obstacles. \'*\' stands for rough terrain. "
		<< "\'@\' stands for points on the path.\n";
	vector<vector<char>> result = grid.getGrid();
	for (Pos i : path) {
		int row = result.size() - i.y;
		int col = i.x - 1;
		result[row][col] = '@';
	}
	Grid::draw(result);
}

vector<Pos> AStar::getPath() const {
	return path;
}

int AStar::getCount() const {
	return counts;
}

double AStar::manhattan(Pos _p, Pos _goal) {
	return abs(_p.x - _goal.x) + abs(_p.y - _goal.y);
}

double AStar::euclidean(Pos _p, Pos _goal) {
	return sqrt(pow(_p.x - _goal.x, 2) + pow(_p.y - _goal.y, 2));
}

void AStar::updateGrid(vector<vector<char>>& _grid, const Pos* _cur, const Pos* _neighbors) {
	if (_cur != nullptr) {
		int row = _grid.size() - _cur->y;
		int col = _cur->x - 1;
		_grid[row][col] = '+';
	}
	if (_neighbors != nullptr) {
		int row = _grid.size() - _neighbors->y;
		int col = _neighbors->x - 1;
		_grid[row][col] = 'o';
	}
}