#include <bits/stdc++.h>
#include <Windows.h>

using namespace std;

const signed inf = 0x3f3f3f3f;

////////////////////////////////////////////////
vector<vector<int>> travel_time;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
double get_wall_time(){
    LARGE_INTEGER time,freq;
    if (!QueryPerformanceFrequency(&freq)){
        //  Handle error
        return 0;
    }
    if (!QueryPerformanceCounter(&time)){
        //  Handle error
        return 0;
    }
    return (double)time.QuadPart / freq.QuadPart;
}
double get_cpu_time(){
    FILETIME a,b,c,d;
    if (GetProcessTimes(GetCurrentProcess(),&a,&b,&c,&d) != 0){
        //  Returns total user time.
        //  Can be tweaked to include kernel times as well.
        return
            (double)(d.dwLowDateTime |
            ((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
    }else{
        //  Handle error
        return 0;
    }
}
////////////////////////////////////////////////////////////////////////
struct ROBOT{
	int curr_charge;
	int curr_cost;
	int mask_drops;
	int mask_pickups;
	int bot_key;
	int max_load;
	vector<int> order_list;

	ROBOT(int bot_k){
		curr_charge = 100;
		curr_cost=0;
		bot_key= bot_k;
		mask_drops=(1<<bot_k);
		mask_pickups=(1<<bot_k);
		order_list = {2*bot_k, 2*bot_k+1};
		max_load=2;
	}
	
	int insertion_cost(int order){         // pseudo insertion cost of an order (NO CHARGING NODE INSERTION)
		int pickup_node = 2*order;
		int drop_node = 2*order+1;
		int cost = inf;             // pseudo_insertion_cost variable to be minimized.
		int n = order_list.size();  // size of the BOT order list currently.
		int load = 0; // load on the BOT (currently taken to be 0).
		
		int best_pickup_cost=inf;

		for(int i=0;i<n;i++){
			if(order_list[i]%2==0){
				load++;
			}else{
				load--;
			}
			if(load>=max_load){
				best_pickup_cost = inf;
				continue;
			} 
			// Initializing cost if the Pickup and Drop both are adjacent to each other ahead of the i-th node.
			cost = min(cost, travel_time[order_list[i]][pickup_node] + travel_time[pickup_node][drop_node] + (i+1<n?travel_time[drop_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0) );
			
			// pickup_cost = cost increment if we insert pickup Node just after i-th node.
			int pickup_cost = travel_time[order_list[i]][pickup_node] + (i+1<n?travel_time[pickup_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			//drop_cost = cost increment if we insert drop Node just after j-th node.
			int drop_cost = travel_time[order_list[i]][drop_node] + (i+1<n?travel_time[drop_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			
			cost = min(cost, drop_cost + best_pickup_cost);
			best_pickup_cost = min(best_pickup_cost, pickup_cost);
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
	int TIME_PER_DELIVERY; // approx time per delivery  (assumed to be in minutes)
	vector<int> deadlines; // deadliness for each order used for tardiness objective.
	vector<int> weights; // higher weights -> higher priority for an order.
	map <string, pair<int,vector<int>> > dp; // DP State // "123" -> 123

	MASTER(){
		MAX_COST=0;
		bots.clear();
		CURR_TIME=0;
	    MAX_CHARGE=100;
	    Q_DOT = 1;  
	    TIME_PER_DELIVERY = 15;
	    deadlines.clear();
		weights.clear();
		dp.clear();
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

	//////////---- Local Search Part: ----///////////////

	void order_swap(int b1, int b2){
		int best_to_swap_b1, best_cost_b1=inf;
		int best_to_swap_b2, best_cost_b2=inf;

		for(int i=2;i<bots[b1].order_list.size();i++){
			if(bots[b1].order_list[i]==-1)continue;
			int new_cost_b2 = bots[b2].insertion_cost(bots[b1].order_list[i]/2);
			if(best_cost_b2 > new_cost_b2){
				best_cost_b2 = new_cost_b2;
				best_to_swap_b1 = bots[b1].order_list[i]/2;
				removal_b1 = travel_time[bots[b1].order_list[i-1]][2*best_to_swap_b1] + (i+1<bots[b1].order_list.size()?travel_time[2*best_to_swap_b1][bots[b1].order_list[i-1]] - travel_time[order_list[i-1]][order_list[i+1]]:0);
			}
		}
		for(int i=2;i<bots[b2].order_list.size();i++){
			if(bots[b2].order_list[i]==-1)continue;
			int new_cost_b1 = bots[b1].insertion_cost(bots[b2].order_list[i]/2);
			if(best_cost_b1 > new_cost_b1){
				best_cost_b1 = new_cost_b1;
				best_to_swap_b2 = bots[b2].order_list[i]/2;
			}
		}

		int removal_b1 = 
		int new_cost_b1 = bots[b1].curr_cost + best_cost_b1 - removal_b1;
		int new_cost_b2 = bots[b2].curr_cost + best_cost_b2 - removal_b2; 

		// IF the state is becoming worse then return;
		int new_avg= (new_cost_b1+new_cost_b2)/2;
		int avg = (bots[b1].curr_cost+bots[b2].curr_cost)/2;
		int new_mx = max(new_cost_b1, new_cost_b2);
		int mx = max(bots[b1].curr_cost, bots[b2].curr_cost);
		if(new_mx - new_avg >= mx-avg)return;

		// We just have to swap best_to_swap_b1 and best_to_swap_b2;
		bots[b1].order_list.insert(bots[b1].order_list.begin()+i2, 2*best_to_swap_b2);
		bots[b1].order_list.insert(bots[b1].order_list.begin()+j2, 2*best_to_swap_b2+1);

		bots[b2].order_list.insert(bots[b2].order_list.begin()+i1, 2*best_to_swap_b1);
		bots[b2].order_list.insert(bots[b2].order_list.begin()+j1, 2*best_to_swap_b1+1);

	}

	//////////-----LOCAL SEARCH ENDS-----///////////

	// For alloting a new order to the optimal BOT.
	void allot(int order){
		int alloted_bot_key = -1, mn_cost = inf;
		for(int i=0;i<bots.size();i++){
			int new_cost = bots[i].insertion_cost(order);
			if(new_cost < mn_cost){
				mn_cost = new_cost;
				alloted_bot_key = i;
			}
		}
		assert(alloted_bot_key!=-1);
		bots[alloted_bot_key].order_list.push_back(2*order);bots[alloted_bot_key].order_list.push_back(2*order+1);
		tsp_update(alloted_bot_key);
		MAX_COST = max(MAX_COST, bots[alloted_bot_key].curr_cost);
	}

	void tsp_update(int bot_key){
		dp.clear();
		sort(bots[bot_key].order_list.begin()+2, bots[bot_key].order_list.end()); // the first 2 nodes are associated with the BOT;
		
		//////////////////////////////
		double cpu0 = get_cpu_time();
    	double wall0 = get_wall_time();
    	//////////////////////////////
	
		tie(bots[bot_key].curr_cost, bots[bot_key].order_list) = tsp(bot_key, bots[bot_key].mask_pickups, bots[bot_key].mask_drops, 2*bot_key+1, CURR_TIME, bots[bot_key].curr_charge); // 1 -> 1st drop node = BOT
		
		/////////////////////////////////
		double cpu1 = get_cpu_time();
		double wall1 = get_wall_time();
		cout<<setprecision(5);
	    cout<<"-----------\nCPU Time : "<<cpu1-cpu0<<" seconds\n";
	    cout<<"Wall Time : "<<wall1-wall0<<" seconds\n-----------\n";
	    /////////////////////////////////


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

		string state = convert_to_string(mask_pickups,  mask_drops,  curr_order,  curr_time,  curr_charge);
		// string closest_state = find_closest_state(mask_pickups, mask_drops, curr_order, curr_time, curr_charge);
		if(dp.find(state)!=dp.end()){
			return	dp[state];
		}

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

};

// Change it for multiple Bots.  -> DONE
// implement closest_node function.  -> DONE
// Accomodate charging in pseudo insertion cost function -> Working on it.
// constraints for priority order alterations. -> R.P

void initialize_travel_time_matrix(int sz){
	for(int i=0;i<sz;i++){
		vector<int> rand;
		for(int j=0;j<sz;j++){
			rand.push_back(uniform_int_distribution<int>(1,10)(rng));
		}
		travel_time.push_back(rand);
	}

	// for(int i=0;i<sz;i++){
	// 	for(int j=0;j<sz;j++){
	// 		cout<<travel_time[i][j]<<" ";
	// 	}cout<<endl;
	// }
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

	// Order Description
	for(int order_id=no_of_bots ; order_id<no_of_orders+no_of_bots ; order_id++){
		
		// double cpu0 = get_cpu_time();
  		// double wall0 = get_wall_time();

		master.deadlines.push_back(0);
		master.weights.push_back(1);
		master.allot(order_id);

		// double cpu1 = get_cpu_time();
	 	// double wall1 = get_wall_time();

		for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=2;j<master.bots[i].order_list.size();j++){
				int x = master.bots[i].order_list[j];
				cout<<(x%2?"Drop":"Pick")<<x/2<<" ";
			}cout<<endl;
		}


	   	// cout<<setprecision(10);
	    // cout<<"-----------\nCPU Time : "<<cpu1-cpu0<<" seconds\n";
	    // cout<<"Wall Time : "<<wall1-wall0<<" seconds\n-----------\n";

	}
	
	// cout<<"Max Cost Recorded : "<<master.MAX_COST<<endl;
	
	// for(int i=0;i<master.bots.size();i++){
	// 	cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
	// 	for(auto x:master.bots[i].order_list){
	// 		cout<<(x%2?"Drop":"Pick")<<x/2<<" ";
	// 	}cout<<endl;
	// }

	// ////////////////////////////////////////////////////////////////

	// double cpu1 = get_cpu_time();
	// double wall1 = get_wall_time();
	// cout<<setprecision(20);
	// cout<<"-----------\nCPU Time : "<<cpu1-cpu0<<" seconds\n";
	// cout<<"Wall Time : "<<wall1-wall0<<" seconds\n-----------";
	    
    return 0;
}




