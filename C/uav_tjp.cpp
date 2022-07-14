#include <iostream>
#include "uav_tjp.h"
#include "point.h"
#include <string>
#include <math.h>

using namespace std;

int Uav_count;
int group_uperbound;
int globEnvDim;

point Action[5] = {point(0, 0), point(0, 1), point(0, -1), point(1, 0), point(-1, 0)};

#define ActionCount ARRLEN(Action)

uav_tjp::uav_tjp(int dim, int grp_uper)
{
	Env_dim = dim;
	globEnvDim = dim;
	Uav_count = 0;
	group_uperbound = grp_uper;
}

uav_tjp::~uav_tjp()
{
}

void uav_tjp::imp_uav(point stt, point prevAct, point dest)
{
	states[Uav_count] = stt;
	prev_action[Uav_count] = prevAct;
	destination[Uav_count] = dest;
	Uav_count++;
}

string uav_tjp::go_forward()
{
	long long int finalActionNumber(0);
	int isGroup[maxUavcount];
	int nAction;

	point UAVActions[maxUavcount];
	point step_action[maxUavcount];

	// start episod
	group g;
	RewardStr best_reward;
	best_reward.setWorst();
	g.update(states);

	for (int a = 0; a < g.count; a++)
	{
		long long int step_action_number(0);
		int tmp_setid = 0;
		for (int b = 0; b < Uav_count; b++)
		{
			if (g.get(b, a))
			{
				isGroup[tmp_setid] = b;
				tmp_setid++;
			}
		}

		// start split to tree
		if (g.gCount(a) > group_uperbound)
		{
			int cost[maxUavcount][maxUavcount];
			for (int j = 0; j < g.gCount(a); j++)
				for (int k = 0; k < g.gCount(a); k++)
					cost[j][k] = states[isGroup[j]].distance(states[isGroup[k]]);

			Tree tree = prime(cost, g.gCount(a), 0);

			RewardStr step_best_reward;
			// step_best_reward.setDefult(Uav_count * -2);
			step_best_reward.setWorst();
			point temp_action[maxUavcount];

			for (int j = 0; j < Uav_count; j++)
				step_action[j] = Action[0];

			Spliter splt = Spliter(g.gCount(a));

			for (int level = 0; level < splt.levels; level++)
			{
				int nodes_count_in_group = splt.level_node_count[level];
				nAction = pow(ARRLEN(Action), nodes_count_in_group);
				int iBestAction = 0;

				for (int j = 0; j < nAction; j++)
				{
					RewardStr rew;
					bool IsValid = true;
					convert(j, Uav_count, temp_action);

					for (int g_mem_counter = 0; g_mem_counter < nodes_count_in_group; g_mem_counter++)
					{
						if (level == 0 && g_mem_counter == 0)
						{
							step_action[0] = temp_action[0];
						}
						else
						{
							step_action[tree.node[(splt.sum_until(level) + g_mem_counter) - 1]] = temp_action[g_mem_counter];
						}
					}

					for (int k = 0; k < g.gCount(a); k++)
						if (!(states[isGroup[k]] + step_action[k]).isValidPoint(Env_dim))
							IsValid = false;

					if (!IsValid)
						continue;

					rew = Reward(prev_action, states, destination, step_action, isGroup, g.gCount(a));

					if (rew.sum(1) > step_best_reward.sum(1))
					{
						step_best_reward = rew;
						iBestAction = j;
					}
					else if (rew.sum(1) == step_best_reward.sum(1))
					{
						if (rew.sum(2) > step_best_reward.sum(2))
						{
							step_best_reward = rew;
							iBestAction = j;
						}
						else if (rew.sum(2) == step_best_reward.sum(2) && rand() % 2 == 0)
						{
							step_best_reward = rew;
							iBestAction = j;
						}
					}
				}

				convert(iBestAction, Uav_count, temp_action);
				for (int g_mem_counter = 0; g_mem_counter < nodes_count_in_group; g_mem_counter++)
				{
					int tttmp = iBestAction / pow(ActionCount, g_mem_counter);
					tttmp = tttmp % ActionCount;
					if (level == 0 && g_mem_counter == 0)
					{
						step_action[0] = temp_action[0];
						step_action_number += tttmp;
					}
					else
					{
						step_action[tree.node[((splt.sum_until(level) + g_mem_counter) - 1)]] = temp_action[g_mem_counter];
						step_action_number += tttmp * pow(ActionCount, tree.node[((splt.sum_until(level) + g_mem_counter) - 1)]);
					}
				}

				/* ************ iBestAction Handle Place     */
			}
		}
		// end split to tree
		else
		{
			nAction = pow(ARRLEN(Action), g.gCount(a));
			RewardStr step_best_reward;
			// step_best_reward.setDefult(Uav_count * -2);
			step_best_reward.setWorst();
			int iBestAction = 0;

			// Check Action Reward's
			for (int j = 0; j < nAction; j++)
			{
				RewardStr rew;
				bool IsValid = true;
				convert(j, Uav_count, step_action);
				for (int k = 0; k < g.gCount(a); k++)
					if (!(states[isGroup[k]] + step_action[k]).isValidPoint(Env_dim))
						IsValid = false;

				if (!IsValid)
					continue;

				rew = Reward(prev_action, states, destination, step_action, isGroup, g.gCount(a));
				if (rew.sum(1) > step_best_reward.sum(1))
				{
					step_best_reward = rew;
					iBestAction = j;
				}
				else if (rew.sum(1) == step_best_reward.sum(1))
				{
					if (rew.sum(2) > step_best_reward.sum(2))
					{
						step_best_reward = rew;
						iBestAction = j;
					}
					else if (rew.sum(2) == step_best_reward.sum(2) && rand() % 2 == 0)
					{
						step_best_reward = rew;
						iBestAction = j;
					}
				}
			}

			step_action_number = iBestAction;
			convert(iBestAction, Uav_count, step_action);

			/* ************ */
		}
		// set final return value

		for (int j = 0; j < g.gCount(a); j++)
		{
			UAVActions[isGroup[j]] = step_action[j];
			int tttmp = step_action_number / pow(ActionCount, j);
			tttmp = tttmp % ActionCount;
			finalActionNumber += tttmp * pow(ActionCount, isGroup[j]);
		}

		/**********/
	}

	string finalRes = "";
	for (int i = 0; i < Uav_count; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if (UAVActions[i] == Action[j])
			{
				finalRes += to_string(j) + ',';
				break;
			}
		}
	} // */

	return finalRes;
}

void group::update(point Stt[])
{
	count = 0;
	for (int i = 0; i < Uav_count; i++)
	{
		setTag[i] = 0;
	}
	for (int i = 0; i < Uav_count; i++)
	{
		if (setTag[i] == 0)
		{
			count++;
			setTag[i] = count;
		}

		for (int j = 0; j < Uav_count; j++)
		{
			if (i != j && Stt[i].distance(Stt[j]) < 4 && setTag[i] != setTag[j])
			{
				if (setTag[j] == 0)
				{
					setTag[j] = setTag[i];
				}
				else
				{
					int t_j = setTag[j];
					int t_i = setTag[i];
					for (int k = 0; k < Uav_count; k++)
					{
						if (setTag[k] == t_j)
						{
							setTag[k] = t_i;
						}
					}
					for (int k = 0; k < Uav_count; k++)
					{
						if (setTag[k] > t_j)
							setTag[k]--;
					}
					count--;
				}
			}
		}
	}
	for (int i = 0; i < Uav_count; i++)
		setTag[i]--;
}

void group::setCount()
{
	int c = 0;
	for (int i = 0; i < Uav_count; i++)
	{
		int j;
		for (j = 0; j < i; j++)
			if (setTag[i] == setTag[j])
				break;

		if (i == j)
			c++;
	}
	count = c;
}

void group::sortTag()
{
	for (int i = 0; i < count; i++)
	{
		bool setExist = false;
		for (int j = 0; j < Uav_count; j++)
			if (setTag[j] == i)
			{
				setExist = true;
				break;
			}
		if (!setExist)
		{
			for (int j = 0; j < Uav_count; j++)
				if (setTag[j] > i)
					setTag[j]--;
			i--;
		}
	}
}

bool group::get(int i, int n)
{
	return (setTag[i] == n);
}

int group::gCount(int i)
{
	int tmp = 0;
	for (int j = 0; j < Uav_count; j++)
		if (setTag[j] == i)
			tmp++;
	return tmp;
}

int RewardStr::sum(int level)
{
	int tmp = 0;
	if (level == 1)
	{
		for (int i = 0; i < Uav_count; i++)
			tmp += R1[i] + R2[i];
		return tmp;
	}
	if (level == 2)
	{
		for (int i = 0; i < Uav_count; i++)
			tmp += R3[i];
		return tmp;
	}
	return 0;
}

void RewardStr::setWorst()
{
	for (int i = 0; i < Uav_count; i++)
	{
		R1[i] = -6;
		R2[i] = -14 * Uav_count * Uav_count;
		R3[i] = -1;
	}
}

void RewardStr::setDefult(int def)
{
	for (int i = 0; i < Uav_count; i++)
	{
		R1[i] = def;
		R2[i] = def;
		R3[i] = def;
	}
}

Tree prime(int cost[][maxUavcount], int n, int s) // n = group.member.count, s= group.id of start node
{
	Tree T;
	int dist[maxUavcount], near[maxUavcount], min_cost = 0;
	bool b[maxUavcount] = {0};
	b[s] = 1;
	for (int i = 0; i < n; i++)
	{
		near[i] = s;
		dist[i] = cost[s][i];
	}
	for (int i = 0; i < n - 1; i++)
	{
		int j = min_edge(b, dist, n);
		b[j] = 1;
		min_cost += dist[j];
		T.near[i] = near[j];
		T.node[i] = j;
		for (int l = 0; l < n; l++)
		{
			if (b[l] == 0 && dist[l] > cost[j][l])
			{
				near[l] = j;
				dist[l] = cost[j][l];
			}
		}
	}
	return T;
}

int min_edge(bool b[], int dist[], int n)
{
	int min = globEnvDim * globEnvDim; // group_uperbound * group_uperbound;
	int id;
	for (int i = 0; i < n; i++)
	{
		if (b[i] == 0)
			if (dist[i] < min)
			{
				min = dist[i];
				id = i;
			}
	}
	return id;
}

Spliter::Spliter(int count)
{
	levels = count / group_uperbound;
	if (count - (levels * group_uperbound) != 0)
	{
		levels++;
	}
	for (int i = 0; i < levels; i++)
	{
		if (levels - i == 1)
			level_node_count[i] = count - ((levels - 1) * group_uperbound);
		else
			level_node_count[i] = group_uperbound;
	}
}

int Spliter::sum_until(int lev)
{
	int temp = 0;
	for (int i = 0; i < lev; i++)
	{
		temp += level_node_count[i];
	}
	return temp;
}

void convert(int iAction, int nUAV, point res[])
{
	for (int k = 0; k < nUAV; k++)
	{ // ( x / 5 ^ i ) mod 5 = Action id of i th UAV
		int tmp = iAction / pow(ActionCount, k);
		tmp = tmp % ActionCount;
		res[k] = Action[tmp];
	}
}

RewardStr Reward(point prev_action[], point Stt[], point destination[], point Act[], int groupList[], int nUAV)
{
	RewardStr Rew;
	for (int i = 0; i < nUAV; i++)
	{
		int dst_dist = Stt[groupList[i]].distance(destination[groupList[i]]);
		int action_dst = (Stt[groupList[i]] + Act[i]).distance(destination[groupList[i]]);
		switch (dst_dist - action_dst)
		{
		case 1:
			Rew.R1[groupList[i]] = 14;
			break;
		case 0:
			Rew.R1[groupList[i]] = 0;
			break;
		case -1:
			Rew.R1[groupList[i]] = -6;
			break;
		default:
			Rew.R1[groupList[i]] = 0;
			break;
		}
		if (dst_dist == 1 && action_dst == 0)
			Rew.R1[groupList[i]] += 6;
	}

	for (int i = 0; i < nUAV; i++)
	{
		for (int j = 0; j < nUAV; j++)
		{
			if (i != j)
			{
				int tmp_dist;
				tmp_dist = (Stt[groupList[i]] + Act[i]).distance(Stt[groupList[j]] + Act[j]);
				if (tmp_dist == 1 && !(Stt[groupList[i]] + Act[i] == destination[groupList[i]]))
					Rew.R2[groupList[i]] -= 3;
				if (tmp_dist == 0)
					Rew.R2[groupList[i]] -= 14 * Uav_count;
			}
		}
	}

	for (int i = 0; i < nUAV; i++)
	{
		Rew.R3[groupList[i]] = direction_rew(prev_action[groupList[i]], Stt[groupList[i]], Act[i]);
	}

	return Rew;
}

int direction_rew(point prev_action, point cur_pos, point act)
{
	if (!(prev_action == point(0, 0)))
	{
		if (prev_action == act) // same direction
			return 2;
		if (prev_action + act == point(0, 0)) // reverse direction
			return -1;
		return 0; // stand or 90
	}
	else // prev_action stand
	{
		point tmpp(0, 0);
		if (tmpp == act)
			return 0;
		return 1;
	}
}
