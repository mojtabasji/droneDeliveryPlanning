#pragma once
#include <iostream>
#include <string>
#include "point.h"

using namespace std;
#define ARRLEN(x) (sizeof(x) / sizeof(x[0]))

const int maxUavcount = 100;

class uav_tjp
{
private:
	/* data */
public:
	int Env_dim;
	point states[maxUavcount];
	point prev_action[maxUavcount];
	point destination[maxUavcount];

	uav_tjp(int dim, int grp_uper);
	~uav_tjp();
	void imp_uav(point stt, point prevAct, point dest);

	string go_forward();
};

class group
{
public:
	int setTag[maxUavcount] = { 0 };
	int count = 0;

	void update(point Stt[]);
	void setCount();
	void sortTag();
	bool get(int i, int n);
	int gCount(int i);
};

class RewardStr
{
public:
	int R1[maxUavcount] = { 0 };
	int R2[maxUavcount] = { 0 };
	int R3[maxUavcount] = { 0 };

	int sum(int level);
	void setWorst();
	void setDefult(int def);
};

class Tree
{
public:
	int near[maxUavcount];
	int node[maxUavcount];
	int count;
	int cost;
};

Tree prime(int cost[][maxUavcount], int n, int s);

int min_edge(bool b[], int dist[], int n);

class Spliter
{
public:
	int level_node_count[maxUavcount];
	int levels;

	Spliter(int count);
	int sum_until(int lev);
};

void convert(int iAction, int nUAV, point res[]);

RewardStr Reward(point prev_action[], point Stt[], point destination[], point Act[], int groupList[], int nUAV);
int direction_rew(point prev_action, point cur_pos, point act);
