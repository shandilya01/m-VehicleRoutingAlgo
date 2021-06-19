#include <bits/stdc++.h>

using namespace std;

// const int n=1+1;  // 1 for the BOT as a NODE

// int travel_time[2*n][2*n]={
// 	{0,0,1,2},
// 	{0,0,1,2},
// 	{1,1,0,2},
// 	{2,2,2,0}
// };

const int n=4+1;  // 1 for the BOT as a NODE

int travel_time[2*n][2*n]={
	{0,0,2,3,4,5,6,7,8,9},
	{0,0,2,3,4,5,6,7,8,9},
	{2,2,0,1,1,1,1,1,1,1},
	{3,3,1,0,1,1,1,1,1,1},
	{4,4,1,1,0,1,1,1,1,1},
	{5,5,1,1,1,0,1,1,1,1},
	{6,6,1,1,1,1,0,1,1,1},
	{7,7,1,1,1,1,1,0,1,1},
	{8,8,1,1,1,1,1,1,0,1},
	{9,9,1,1,1,1,1,1,1,0},
};

// time taken from one node to another after visiting closest charging station midway
// int charge_time[2*n][2*n]={
// 	{0,1,3,4,5,6,7,8,9,10},
// 	{1,0,3,4,5,6,7,8,9,10},
// 	{3,3,0,1,1,1,1,1,1,1},
// 	{4,4,0,0,1,1,1,1,1,1},
// 	{5,5,0,1,0,1,1,1,1,1},
// 	{6,6,0,1,1,0,1,1,1,1},
// 	{7,7,0,1,1,1,0,1,1,1},
// 	{8,8,0,1,1,1,1,0,1,1},
// 	{9,9,0,1,1,1,1,1,0,1},
// 	{10,10,0,1,1,1,1,1,1,0},
// };

int CURR_TIME=0;
int CURR_CHARGE=100;
const int MAX_TIME = 100;
const int MAX_CHARGE = 100;
const int q_dot = 5;  // units of charge consumed per unit of time

// deadlines for each order
int deadline[n+1];

// weights for each order i.e higher weight -> higher priority
int weight[n+1];

// Cost that increases with time and rate depends on the order weight and deadline.
int delivery_cost(int loc, int time){
	return weight[loc]*max(0, time-deadline[loc]) + weight[loc]*max(0,time-deadline[loc]/2);
}	

// returns the best possible charge and time when we reach the next node.
pair<int,int> best_station(int curr_order, int next_order, int curr_charge){
	return make_pair(100, travel_time[curr_order][next_order]);
}
	
// DP State
map <string, pair<int,vector<int>> > dp;	

// Parent Vector
int par[2*n+1]={};

string convert_to_string(int a, int b, int c, int d, int e){
	return to_string(a)+"#"+to_string(b)+"#"+to_string(c)+"#"+to_string(d)+"#"+to_string(e);
}

// Time Complexity : O(n*(4^n)*MAX_TIME*MAX_CHARGE) where n is no of orders made
// Time Complexity : O(n*(4^n)) where n is no of orders made
pair< int,vector<int> > tsp(int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
	int pickups = __builtin_popcount(mask_pickups);
	int drops = __builtin_popcount(mask_drops);
	int load = pickups - drops;
	assert((load>=0 && load<=2));
	if(drops == n){
		return make_pair(0, vector<int> ());
	}

	string state = convert_to_string( mask_pickups,  mask_drops,  curr_order,  curr_time,  curr_charge);
	string closest_state = state;//closest_state(state);
	if(dp.find(closest_state)!=dp.end()){
		return	dp[closest_state];
	}

	int ans = INT_MAX;
	vector<int> opt_path;
	// going for another pickup if load<=1;
	if(load<=1){
		for(int next_order=0;next_order<2*n;next_order+=2){
			int loc = (next_order)/2;
			int next_charge = curr_charge - q_dot*travel_time[curr_order][next_order];
			int next_time = curr_time+travel_time[curr_order][next_order];
			if((mask_pickups&(1<<loc))==0 and next_charge>0){
				pair<int,vector<int>> val = tsp((mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
				int newMin = travel_time[curr_order][next_order] + val.first;
				if(ans>newMin){
					ans=newMin;
					vector<int> dum = val.second;
					dum.push_back(next_order);
					opt_path  = dum;
				}
			}
		}
	}	
	// going for a drop if load>0 AND adding a delivery cost for passing deadlines
	if(load>0){
		for(int next_order=1;next_order<2*n;next_order+=2){
			int loc = (next_order)/2;
			int next_charge = curr_charge - q_dot*travel_time[curr_order][next_order];
			int next_time = curr_time+travel_time[curr_order][next_order];
			if((mask_drops&(1<<loc))==0 and (mask_pickups&(1<<loc))>0  and next_charge>0){
				pair<int,vector<int>> val = tsp(mask_pickups, (mask_drops|(1<<loc)), next_order, next_time, next_charge);
				int newMin = travel_time[curr_order][next_order] + val.first + delivery_cost(loc, next_time);
				if(ans>newMin){
					ans=newMin;
					vector<int> dum = val.second;
					dum.push_back(next_order);
					opt_path = dum;
				}
			}
		}
	}
	//going for a charging station ONLY if load==0
	if(load==0){
		for(int next_order=0;next_order<2*n;next_order+=2){
			int loc = (next_order)/2;
			int next_charge,next_time;
			tie(next_charge, next_time) = best_station(curr_order, next_order, curr_charge);
			next_time+=curr_time;
			if((mask_pickups&(1<<loc))==0 and next_charge>0){
				pair<int,vector<int>> val = tsp((mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
				int newMin = travel_time[curr_order][next_order] + val.first;
				if(ans>newMin){
					ans=newMin;
					vector<int> dum = val.second;
					dum.push_back(next_order);
					opt_path  = dum;
				}
			}
		}
	}
	return dp[state] = make_pair(ans, opt_path);
}

signed main(){
	#ifndef ONLINE_JUDGE 
	freopen("input.txt", "r", stdin); 
	freopen("output.txt", "w", stdout); 
	#endif

	for(int i=0;i<n+1;i++){
		deadline[i] = i-CURR_TIME;
		weight[i]=1;
	}

	pair<int,vector<int>> path = tsp(1,1,1, CURR_TIME, CURR_CHARGE);
	reverse(path.second.begin(), path.second.end());
	cout<<"Cost for Minimum path : "<<path.first<<endl;
	cout<<"Visiting Optimal Order : ";
	for(int i=0;i<path.second.size();i++){
		cout<<((path.second[i])%2==1?"D":"P")<<path.second[i]/2<<" ";
	}
	cout<<endl;

    return 0;
}


