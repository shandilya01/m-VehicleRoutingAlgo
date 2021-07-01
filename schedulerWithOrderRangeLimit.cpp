#include <bits/stdc++.h>

using namespace std;

const signed inf = 0x3f3f3f3f;

////////////////////////////////////////////////
vector<vector<int>> travel_time;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

struct ROBOT{
	int curr_charge;
	int curr_cost;
	int mask_drops;
	int mask_pickups;
	vector<int> order_list;
	vector<int>order_list_with_constraints;
	int load_of_bot;
	
	ROBOT(int bot_key){
		curr_charge = 100;
		curr_cost=0;
		mask_drops=(1<<bot_key);
		mask_pickups=(1<<bot_key);
		order_list = {2*bot_key, 2*bot_key+1};
		load_of_bot=0;
	}

	int insertion_cost(int order){         // pseudo insertion cost in order no = order
		int pickup_node = 2*order;
		int drop_node = 2*order+1;
		int cost = INT_MAX;             // pseudo_insertion_cost variable to be minimized.
		int n = order_list.size();  // size of the BOT order list currently.
		int load = 0; // load on the BOT (currently taken to be 0).

		for(int i=0;i<n;i++){
			if(order_list[i]%2==0){
				load++;
			}else{
				load--;
			} 
			// Initializing cost if the Pickup and Drop both are adjacent to each other ahead of the i-th node.
			cost = min(cost, travel_time[order_list[i]][pickup_node] + travel_time[pickup_node][drop_node] + (i+1<n?travel_time[drop_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0) );
			
			// pickup_cost = cost increment if we insert pickup Node just after i-th node.
			int pickup_cost = travel_time[order_list[i]][pickup_node] + (i+1<n?travel_time[pickup_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			
			int j=i+1;
			int copy_load = load;
			while(j<n and load<=1){
				//drop_cost = cost increment if we insert drop Node just after j-th node.
				int drop_cost = travel_time[order_list[j]][drop_node] + (j+1<n?travel_time[drop_node][order_list[j+1]] - travel_time[order_list[j]][order_list[j+1]]:0);
				
				// total_cost of insertion  = pickup_cost + drop_cost  thus finding the minimum of that.
				cost = min(cost , pickup_cost + drop_cost);
				j++;

				// Updating load var
				if(order_list[j]%2==0){
					load++;
				}else{
					load--;
				} 
			}

			load = copy_load;
		}
		return cost;
	}
	
};

struct MASTER{
	int MAX_COST;  // maximum cost over all the costs of ROBOTS 
	vector<ROBOT> bots; // list of all ROBOT objects.
	int CURR_TIME;  // current time
	int MAX_CHARGE; // maximum charging amount
	int Q_DOT;  // units of charge consumed per unit of time
	vector<int> deadlines; // deadliness for each order used for tardiness objective.
	vector<int> weights; // higher weights -> higher priority for an order.
	map <string, pair<int,vector<int>> > dp; // DP State
	int current_load_threshold;
	map<int,pair<int,int>>movable_positions;
	
	MASTER(){
		MAX_COST=0;
		bots.clear();
		CURR_TIME=0;
	    MAX_CHARGE=100;
	    Q_DOT = 1;  
	    deadlines.clear();
		weights.clear();
		dp.clear();
		current_load_threshold=0;
	}

	// Cost that increases with time and rate depends on the order weights and deadlines.
	int delivery_cost(int order, int curr_time){
		return weights[order]*max(0, curr_time-deadlines[order]) + weights[order]*max(0, curr_time-deadlines[order]/2);
	}	

	// returns the best possible charge amount and time when we reach the next node.
	pair<int,int> best_station(int curr_order, int next_order, int curr_charge, int curr_time){
		return make_pair(100, curr_time+travel_time[curr_order][next_order]);
	}

	// HELPER function for conversion of a state to string.
	string convert_to_string(int a, int b, int c, int d, int e){
		return to_string(a)+"#"+to_string(b)+"#"+to_string(c)+"#"+to_string(d)+"#"+to_string(e);
	}

	// Returns the closest state : if found in the search space
	string find_closest_state(int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
		
		int time_tolerance = 5;
		int charge_tolerance = 5;

		for(int ch=max(0,curr_charge-charge_tolerance);ch<min(MAX_CHARGE, curr_charge+charge_tolerance);ch++){
			for(int time=max(0,curr_time-time_tolerance);time<curr_time+time_tolerance;time++){
				string cl_state = to_string(mask_pickups)+"#"+to_string(mask_drops)+"#"+to_string(curr_order)+"#"+to_string(time)+"#"+to_string(ch);
				if(dp.find(cl_state)!=dp.end()){
					return cl_state;
				}
			}
		}

		return "?";
	}

	// For alloting a new order to the optimal BOT.
	void allot(int order,int no_of_bots,int no_of_orders){
		current_load_threshold=(no_of_orders/no_of_bots)+1;
		int alloted_bot_key = -1, mn_cost = INT_MAX;
		for(int i=0;i<bots.size();i++){
			if(bots[i].load_of_bot<=current_load_threshold)
			{
			int new_cost = bots[i].insertion_cost(order);
			if(new_cost < mn_cost){
				mn_cost = new_cost;
				alloted_bot_key = i;
			}
			}
		}
		assert(alloted_bot_key!=-1);
		bots[alloted_bot_key].order_list.push_back(2*order);bots[alloted_bot_key].order_list.push_back(2*order+1);
		bots[alloted_bot_key].load_of_bot++;
		tsp_update(alloted_bot_key);
		MAX_COST = max(MAX_COST, bots[alloted_bot_key].curr_cost);
	}

	void tsp_update(int bot_key){
		dp.clear();
		sort(bots[bot_key].order_list.begin()+2, bots[bot_key].order_list.end()); // the first 2 nodes are associated with the BOT;
		tie(bots[bot_key].curr_cost, bots[bot_key].order_list) = tsp(bot_key, bots[bot_key].mask_pickups, bots[bot_key].mask_drops, 2*bot_key+1, CURR_TIME, bots[bot_key].curr_charge); // 1 -> 1st drop node = BOT
		bots[bot_key].order_list.push_back(2*bot_key+1),bots[bot_key].order_list.push_back(2*bot_key);
		reverse(bots[bot_key].order_list.begin(), bots[bot_key].order_list.end());
	}

	pair< int,vector<int> > tsp(int bot_key, int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
		int n=0;
		for(int x:bots[bot_key].order_list){
			if(x!=-1)n++;
		}
		n/=2;

		int pickups = __builtin_popcount(mask_pickups);
		int drops = __builtin_popcount(mask_drops);
		int load = pickups - drops;
		assert((load>=0 && load<=2));
		if(drops == n){
			assert(pickups == n);
			return make_pair(0, vector<int> ());
		}

		string closest_state = find_closest_state(mask_pickups, mask_drops, curr_order, curr_time, curr_charge);
		if(closest_state!="?"){
			return	dp[closest_state];
		}
		string state = convert_to_string(mask_pickups,  mask_drops,  curr_order,  curr_time,  curr_charge);

		int ans = inf;
		vector<int> opt_path;

		// going for another pickup if load<=1;
		if(load<=1){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time + travel_time[curr_order][next_order];
				if((mask_pickups&(1<<loc))==0 and next_charge>0){
					pair<int,vector<int>> val = tsp(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
					
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

		// going for a drop if load>0 AND adding a tardiness cost for passing deadlines
		if(load>0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==0)continue;  // not a drop node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time+travel_time[curr_order][next_order];
				if((mask_drops&(1<<loc))==0 and (mask_pickups&(1<<loc))>0  and next_charge>0){
					pair<int,vector<int>> val = tsp(bot_key, mask_pickups, (mask_drops|(1<<loc)), next_order, next_time, next_charge);
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

		// going for a charging station ONLY if load==0
		if(load==0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				int next_charge,next_time;
				tie(next_charge, next_time) = best_station(curr_order, next_order, curr_charge, curr_time);
				if((mask_pickups&(1<<loc))==0 and next_charge>0){
					pair<int,vector<int>> val = tsp(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
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


	// For alloting a new order to the optimal BOT.
	void allot_with_constraints(int order,int no_of_bots,int no_of_orders){
		for(int i=0;i<bots.size();i++)
			{
				bots[i].load_of_bot=0;
				bots[i].curr_cost=0;
			}
		current_load_threshold=(no_of_orders/no_of_bots)+1;
		int alloted_bot_key = -1, mn_cost = INT_MAX;
		for(int i=0;i<bots.size();i++){
			if(bots[i].load_of_bot<=current_load_threshold)
			{
			int new_cost = bots[i].insertion_cost(order);
			if(new_cost < mn_cost){
				mn_cost = new_cost;
				alloted_bot_key = i;
			}
			}
		}
		assert(alloted_bot_key!=-1);
		bots[alloted_bot_key].order_list_with_constraints.push_back(2*order);bots[alloted_bot_key].order_list_with_constraints.push_back(2*order+1);
		bots[alloted_bot_key].load_of_bot++;
		tsp_with_constraints_update(alloted_bot_key);
		MAX_COST = max(MAX_COST, bots[alloted_bot_key].curr_cost);
	}

	void tsp_with_constraints_update(int bot_key){
		dp.clear();
		sort(bots[bot_key].order_list_with_constraints.begin()+2, bots[bot_key].order_list_with_constraints.end()); // the first 2 nodes are associated with the BOT;
		tie(bots[bot_key].curr_cost, bots[bot_key].order_list_with_constraints) = tsp(bot_key, bots[bot_key].mask_pickups, bots[bot_key].mask_drops, 2*bot_key+1, CURR_TIME, bots[bot_key].curr_charge); // 1 -> 1st drop node = BOT
		bots[bot_key].order_list_with_constraints.push_back(2*bot_key+1),bots[bot_key].order_list_with_constraints.push_back(2*bot_key);
		reverse(bots[bot_key].order_list_with_constraints.begin(), bots[bot_key].order_list_with_constraints.end());
	}

	pair< int,vector<int> > tsp_with_constraints(int bot_key, int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
		int n=0;
		for(int x:bots[bot_key].order_list){
			if(x!=-1)n++;
		}
		n/=2;

		int pickups = __builtin_popcount(mask_pickups);
		int drops = __builtin_popcount(mask_drops);
		int load = pickups - drops;
		assert((load>=0 && load<=2));
		if(drops == n){
			assert(pickups == n);
			return make_pair(0, vector<int> ());
		}

		string closest_state = find_closest_state(mask_pickups, mask_drops, curr_order, curr_time, curr_charge);
		if(closest_state!="?"){
			return	dp[closest_state];
		}
		string state = convert_to_string(mask_pickups,  mask_drops,  curr_order,  curr_time,  curr_charge);

		int ans = inf;
		vector<int> opt_path;

		// going for another pickup if load<=1;
		if(load<=1){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time + travel_time[curr_order][next_order];
				if((mask_pickups&(1<<loc))==0 and next_charge>0 ){
					pair<int,vector<int>> val = tsp_with_constraints(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
					
					if(movable_positions[curr_order].first<=val.second.size() and movable_positions[curr_order].second>=val.second.size())
					{
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
		}	

		// going for a drop if load>0 AND adding a tardiness cost for passing deadlines
		if(load>0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==0)continue;  // not a drop node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time+travel_time[curr_order][next_order];
				if((mask_drops&(1<<loc))==0 and (mask_pickups&(1<<loc))>0  and next_charge>0){
					pair<int,vector<int>> val = tsp_with_constraints(bot_key, mask_pickups, (mask_drops|(1<<loc)), next_order, next_time, next_charge);
					
					if(movable_positions[curr_order].first<=val.second.size() and movable_positions[curr_order].second>=val.second.size())
					{
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
		}

		// going for a charging station ONLY if load==0
		if(load==0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				int next_charge,next_time;
				tie(next_charge, next_time) = best_station(curr_order, next_order, curr_charge, curr_time);
				if((mask_pickups&(1<<loc))==0 and next_charge>0){
					pair<int,vector<int>> val = tsp_with_constraints(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
					if(movable_positions[curr_order].first<=val.second.size() and movable_positions[curr_order].second>=val.second.size())
					{					
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
		}

		return dp[state] = make_pair(ans, opt_path);
	}
};

// Change it for multiple Bots.  -> DONE
// implement closest_node function.  -> DONE
// Accomodate charging in pseudo insertion cost function -> Working on it.
// constraints for priority order alterations. -> R.P

void initialize_travel_time_matrix(int sz){
	for(int i=0;i<sz;i++){
		vector<int> rand;
		for(int j=0;j<sz;j++){
			rand.push_back(uniform_int_distribution<int>(1,100)(rng));
		}
		travel_time.push_back(rand);
	}

	// for(int i=0;i<sz;i++){
	// 	for(int j=0;j<sz;j++){
	// 		cout<<travel_time[i][j]<<" ";
	// 	}cout<<endl;
	// }
}


void print_schedule(MASTER master,int with_constraints)
{
	if(with_constraints==0)
	{
		cout<<"Updated Lists :\n";
		for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=2;j<master.bots[i].order_list.size();j++){
				int x = master.bots[i].order_list[j];
				cout<<(x%2?"Drop":"Pick")<<x/2<<" ";
			}cout<<endl;
		}	
	}
	if(with_constraints==1)
	{
		cout<<"Updated Lists :\n";
		for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=2;j<master.bots[i].order_list_with_constraints.size();j++){
				int x = master.bots[i].order_list_with_constraints[j];
				cout<<(x%2?"Drop":"Pick")<<x/2<<" ";
			}cout<<endl;
		}
	}
}
void scheduling_without_constraints(MASTER &master,int no_of_orders,int no_of_bots,int output_schedule)
{
		// Order Description
	for(int order_id=no_of_bots ; order_id<no_of_orders+no_of_bots ; order_id++){
		
		master.deadlines.push_back(0);
		master.weights.push_back(1);
		master.allot(order_id,no_of_bots,no_of_orders);
		if(output_schedule==1)
		{
			print_schedule(master,0);
		}
	}
}

void scheduling_with_constraints(MASTER &master,int no_of_orders,int no_of_bots)
{
	int range=0;
	scheduling_without_constraints(master,no_of_orders,no_of_bots,0);
	for(int i=0;i<master.bots.size();i++){
		for(int j=2;j<master.bots[i].order_list.size();j++){
			int order_list_value = master.bots[i].order_list[j];
			cout<<"Enter range of movable positions for  "<<(order_list_value%2?"Drop":"Pick")<<order_list_value/2<<" "<<" for bot "<<i<<"\n";
			cin>>range;
			master.movable_positions[order_list_value]={j-range,j+range};
			}
		}
	for(int order_id=no_of_bots ; order_id<no_of_orders+no_of_bots ; order_id++){
		
		master.deadlines.push_back(0);
		master.weights.push_back(1);
		master.allot_with_constraints(order_id,no_of_bots,no_of_orders);
			print_schedule(master,1);
	}
}
signed main(){

	// double cpu0 = get_cpu_time();
    // double wall0 = get_wall_time();

   	/////////////////////////////////////////////////////////////////

	MASTER master;
	cout<<"No of BOTS:\n";
	int no_of_bots;
	cin>>no_of_bots;

	// BOT Description
	for(int bot_id=0;bot_id<no_of_bots;bot_id++){
		ROBOT robot(bot_id);
		master.bots.push_back(robot);
		master.deadlines.push_back(0);
		master.weights.push_back(1);
	}

	cout<<"No of Orders:\n";
	int no_of_orders;
	cin>>no_of_orders;

	// randomly fills the travel_time matrix with time from each node to another.
	initialize_travel_time_matrix(2*(no_of_orders + no_of_bots));
	
	int constraint_variable=0;
	cout<<"Enter if constraints are available \n";
	cin>>constraint_variable;
	if(constraint_variable==0)
	{
	scheduling_without_constraints(master,no_of_orders,no_of_bots,1);
	}
	else if(constraint_variable==1)
	{
	scheduling_with_constraints(master,no_of_orders,no_of_bots);
	}
    
    return 0;
}