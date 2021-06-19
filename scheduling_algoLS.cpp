#include <bits/stdc++.h>

using namespace std;

const int m=2;

// Orders are alloted to the nearest BOT from PICKUP.

// for i-th order ; pickup -> (2*i)-th ; drop-> (2*i+1)-th.

vector<int> order_list[m];
int pickups_state[m];
int drops_state[m];
int curr_costs[m];

// performs a string_exchange move on BOTs i and j.
void string_exchange(int b1, int b2){
	int n = order_list[b1].size();
	int m = order_list[b2].size();
	vector<int> a = order_list[b2];
	vector<int> b = order_list[b1];
	sort(a.begin(), a.end());
	sort(b.begin(), b.end());

	int ans = max(curr_costs[b1], curr_costs[b2]);
	for(int h=0;h<n;h+=2){
		if((pickups_state[b1]&(1<<(h/2)))>0)continue;
		for(int k=0;k<m;k+=2){
			if((pickups_state[b2]&(1<<(k/2)))>0)continue;
			swap(a[h], b[k]);swap(a[h+1],b[k+1]);
			int cost_b1 = tsp_update(a);
			int cost_b2 = tsp_update(b);

			if( ans > max(cost_b1, cost_b2) ){
				ans = max(cost_b1, cost_b2);
				curr_costs[b1] = cost_b1;
				curr_costs[b2] = cost_b2;
				order_list[b1] = a;
				order_list[b2] = b;
			}
			sort(a.begin(), a.end());
			sort(b.begin(), b.end());
		}
	}	
}

// String Replacement + Exchange 
void string_mix(int b1, int b2){
	if(curr_costs[b1]<curr_costs[b2]){
		swap(b1,b2);
	}
	//cost_b1 > cost_b2 means b1-1 -> b2+1 replacement
	int n = order_list[b1].size();
	int m = order_list[b2].size();

	vector<int> aa = order_list[b1];
	vector<int> bb = order_list[b2];
	sort(aa.begin(), aa.end());
	sort(bb.begin(), bb.end());
	vector<int> a;
	vector<int> b;

	int kappa = 0;

	int ans = max(curr_costs[b1], curr_costs[b2]);
	for(int h=0;h<n and kappa<2;h+=2){
		if((pickups_state[b1]&(1<<(h/2)))>0)continue;

		a=aa;b=bb;
		// Replacing orders.
		b.push_back(a[h]);b.push_back(a[h+1]);
		a.erase(a.begin()+h, a.begin()+h+2);

		int cost_b1 = tsp_update(a);
		int cost_b2 = tsp_update(b);

		// if we do a replacement from a->b operation.
		if(ans > max(cost_b1, cost_b2)){
			kappa++;
			ans = max(cost_b1, cost_b2);
			curr_costs[b1] = cost_b1;
			curr_costs[b2] = cost_b2;
			order_list[b1] = a;
			order_list[b2] = b;
		}

		for(int k=0;k<m and kappa<2;k+=2){
			if((pickups_state[b2]&(1<<(k/2)))>0)continue;

			// re-initialization 
			a=aa;b=bb;

			// Exchanging orders.
			swap(a[h], b[k]);swap(a[h+1], b[k+1]);

			int cost_b1 = tsp_update(a);
			int cost_b2 = tsp_update(b);

			// if we do an exchange operation. 
			if( ans > max(cost_b1, cost_b2) ){
				kappa++;
				ans = max(cost_b1, cost_b2);
				curr_costs[b1] = cost_b1;
				curr_costs[b2] = cost_b2;
				order_list[b1] = a;
				order_list[b2] = b;
			}
		}
	}	
}

// updates the ordering for i-th BOT to find optimal route and returns order and cost.
void tsp_update(vector<int> a){
	
}

// Assuming that the order_list contains BOT NODE on the 0-th index ;
void pseudo_insertion_cost(int bot, int pickup_node, int drop_node){	
	int cost=LLONG_MAX;              // pseudo_insertion_cost variable to be minimized.
	int n = order_list[bot].size();  // size of the BOT order list currently.
	int load = 0; // load on the BOT (currently taken to be 0).

	for(int i=0;i<n;i++){
		if(order_list[bot][i]%2==0){
			load++;
		}else{
			load--;
		} 
		// Initializing cost if the Pickup and Drop both are adjacent to each other ahead of the i-th node.
		cost = min(cost, travel_time[order_list[bot][i]][pickup_node] + travel_time[pickup_node][drop_node] + (i+1<n?travel_time[drop_node][order_list[bot][i+1]] - travel_time[order_list[bot][i]][order_list[bot][i+1]]:0) );
		
		// pickup_cost = cost increment if we insert pickup Node just after i-th node.
		int pickup_cost = travel_time[order_list[bot][i]][pickup_node] + (i+1<n?travel_time[pickup_node][order_list[bot][i+1]] - travel_time[order_list[bot][i]][order_list[bot][i+1]]:0);
		
		int j=i+1;
		int copy_load = load;
		while(j<n and load<=1){
			//drop_cost = cost increment if we insert drop Node just after j-th node.
			int drop_cost = travel_time[order_list[bot][j]][drop_node] + (j+1<n?travel_time[drop_node][order_list[bot][j+1]] - travel_time[order_list[bot][j]][order_list[bot][j+1]]:0);
			
			// total_cost of insertion  = pickup_cost + drop_cost  thus finding the minimum of that.
			cost = min(cost , pickup_cost + drop_cost);
			j++;

			// Updating load var
			if(order_list[bot][j]%2==0){
				load++;
			}else{
				load--;
			} 
		}

		load = copy_load;
	}
	return cost;
}

signed main(){
	#ifndef ONLINE_JUDGE 
	freopen("input.txt", "r", stdin); 
	freopen("output.txt", "w", stdout); 
	#endif

	for(int i=0;i<m;i++){
		for(int j=i+1;j<m;j++){
			string_exchange(i,j);
		}
	}

    return 0;
}