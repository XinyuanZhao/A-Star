#ifndef AStar_H
#define AStar_H

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <stack>
#include <functional>
#include <cmath>
#include <assert.h>
using namespace std;

struct Pos {
	int x, y;
	Pos operator + (const Pos p) const { return Pos{ x + p.x, y + p.y }; }
	bool operator == (const Pos p) const { return p.x == x && p.y == y; }
	bool operator != (const Pos p) const { return !(*this == p); }
	bool operator < (const Pos p) const { return tie(x, y) < tie(p.x, p.y); }
};

struct Node {
	Pos position;
	double priority;
	int moves;
};

struct cmp {
	bool operator()(Node a, Node b) {
		if (a.priority > b.priority) return true;
		if (a.priority == b.priority && a.moves < b.moves) return true;
		return false;
	}
};

class Grid {
public:
	Grid();
	Grid(int _h, int _w, double _roughTerrainCost = 5, double _diagCost = 1.4);
	void setObstacle(const Pos _tl, const Pos _br);
	void clearObstacle();
	void setRoughTerrain(const Pos _tl, const Pos _br);
	void clearRoughTerrain();
	void setRoughTerrainCost(const double _cost);
	void setDiagCost(const double _cost);
	void enableDiagMove(const bool _flag);
	int getHeight() const;
	int getWidth() const;
	vector<Pos> neighbors(const Pos _p) const;
	vector<vector<char>> getGrid() const;
	double getCost(const Pos _from, const Pos _to) const;
	bool isValid(const Pos _p) const;
	void draw() const;
	static void draw(const vector<vector<char>>& _grid);

private:
	int height, width;
	double roughTerrainCost;
	double diagCost;
	set<Pos> obstacle;
	set<Pos> roughTerrain;
	int dir;
	static vector<Pos> DIRS;
};

class AStar {
public:
	AStar() {};
	AStar(const Grid _grid, const Pos _start, const Pos _goal);
	void setHeuristic(const function<double(Pos, Pos)> _H);
	void solve();
	void drawSeq() const;
	void drawPath() const;
	vector<Pos> getPath() const;
	int getCount() const;

private:
	Grid grid;
	Pos start, goal;
	int counts;
	priority_queue<Node, vector<Node>, cmp> frontier;
	vector<Pos> path;
	vector<vector<vector<char>>> gridSeq;
	map<Pos, Pos> parent;
	map<Pos, double> cost;
	function<double(Pos, Pos)> heuristic;
	static double manhattan(Pos _p, Pos _goal);
	static double euclidean(Pos _p, Pos _goal);
	static void updateGrid(vector<vector<char>>& _grid, const Pos* _cur, const Pos* _neighbors);
};
#endif // !AStar_H