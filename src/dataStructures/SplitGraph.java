package dataStructures;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;

import pulseStructures.PulseHandler;
import pulseStructures.PulseNode;

/**
 * This class represents a split graph for the set of instances with a depot.
 */

public class SplitGraph {

	/**
	 * Predecessors list
	 */
	
	private Integer[] p;
	
	/**
	 * Labels list
	 */
	
	private Double[] c;
	
	/**
	 * Number of nodes on the split graph = clients+1
	 */
	private int N;
	
	/**
	 * Number of routes in the final solution
	 */
	private int numRoutes;
	
	/**
	 * A list of id's from the optimal routes
	 */
	
	private ArrayList<Integer> optimalRoutes;
	
	/**
	 * A hashtable to store a character with the whole route (for easy checking)
	 */
	
	private Hashtable<String,String> routeStrings;
	
	/**
	 * A hashtable to store a character with the whole route with the format for the online app.
	 */
	private Hashtable<String,ArrayList<String>> routePrints;
	
	/**
	 * The time for a given route.
	 */
	private Hashtable<String,Double> routeTimes;
	
	/**
	 * The cost for a given route.
	 */
	private Hashtable<String,Double> routeCosts;
	
	/**
	 * The number assigned to each route.
	 */
	private Hashtable<String,Integer> routeIDs;
	
	/**
	 * The id of the heuristic that originated the tsp:
	 */
	
	private int heur;
	
	/**
	 * Creates a split graph
	 * @param tsp
	 * @param g
	 * @param data
	 * @throws IOException 
	 */
	public SplitGraph(ArrayList<Integer> tsp, OriginalGraph g, DataHandler data,int routes,int heu) throws IOException {

		N = tsp.size()+1;
		p = new Integer[N];
		c = new Double[N];
		numRoutes = routes;
		heur = heu;
		optimalRoutes = new ArrayList<Integer>();
		routeStrings = new Hashtable<String,String>();
		routePrints = new Hashtable<String,ArrayList<String>>();
		routeTimes = new Hashtable<String,Double>();
		routeCosts = new Hashtable<String,Double>();
		routeIDs = new Hashtable<String,Integer>();
		if(N >= 9999 && N <=10000) { //This is not currently applied but it could be a good improvement!
			this.runSP2(tsp, g, data);
			this.recoverSP2(data,tsp);
		}else {
			this.runSP1(tsp, g, data);
			this.recoverSP(data,tsp);
			
		}
		
	}
	
	/**
	 * This method runs a shortest path while creating the graph as on the BF alg
	 * @throws IOException 
	 */
	public void runSP1(ArrayList<Integer> tsp,OriginalGraph G,DataHandler dataHandler) throws IOException{
		
		//0. Initialize all labels:
		
			c[0] = 0.0;
			for(int i=1;i<N;i++){
				c[i] = 999999.0;
				G.getDists()[i-1][i-1] = 0.0;
				G.getwTimes()[i-1][i-1] = 0.0;
				G.getdTimes()[i-1][i-1] = 0.0;
			}
			
		//1. First loop: All split nodes
			
			for(int i=0;i<N-1;i++){ 	//(Tail)
				
				//Check a route where only the current node is visited:
				
					int j = i+1; 		//(Head)
					double[] weights = new double[2];
					weights[0] = 0.0; //cost is 0
					weights[1] = dataHandler.getServicesTimes().get(tsp.get(j-1)) + dataHandler.getParkTime(); //time
					
					if(dataHandler.getFixCost()+c[i] < c[j]){
						c[j] = dataHandler.getFixCost() + c[i];
						p[j] = i;
						//Add's the route:
						routeStrings.put((i+";"+j),"-> "+tsp.get(j-1));
						routePrints.put((i+";"+j), new ArrayList<String>());
						routePrints.get((i+";"+j)).add(tsp.get(j-1)+";"+tsp.get(j-1)+"1");
						routeTimes.put((i+";"+j),weights[1]);
						routeCosts.put((i+";"+j),weights[0]+dataHandler.getFixCost());
					}
				
				//The first head:
				
					j = i+2; 		//(Head)
				
				//Creates a pulse handler (an empty board for the pulse):
				
					PulseHandler pH = new PulseHandler(dataHandler,G,tsp);
				
				// A counter to know if the network must be created from scratch or not:
					
					int cont = 1;
					boolean terminar = false;
					
				//2. Second loop: all possible head nodes:
					
					tsp.add(i,N-1);
			
					while(!terminar && j<N){
						
							terminar = true;	
							
							double[] newWeights = new double[2];
						
							if(cont == 1) {
								newWeights = calculateDRWW(i, j+1,pH,dataHandler,G,tsp);
								cont++;
							}else {
								newWeights = calculateDRWWRep(i, j+1,pH,dataHandler,G,tsp);
							}
							
							//Evaluates if the arc is better:
							
							if(newWeights[1] < 999999){
								
								//We must continue:
								
								terminar = false;
								
								if(newWeights[0]+ c[i] < c[j]) {
									
									//Adds the route:
									
									routeStrings.put((i+";"+j),recoverRoute(pH.getPath(),dataHandler,numRoutes,tsp,(i+";"+j)));
									routeTimes.put((i+";"+j),newWeights[1]);
									routeCosts.put((i+";"+j),newWeights[0]);
									
								//Updates labels:
								
									c[j] = newWeights[0]+ c[i];
									p[j] = i;
								
								
								}
								
							}

							//Update j
							j+=1;
					}
				
					tsp.remove(i);
				
			}

		
	}
	
	/**
	 * This method runs a shortest path while creating the graph as on the BF alg.
	 * The main difference with runSP1 is that routes are added every time a label is updated.
	 * @throws IOException 
	 */
	public void runSP2(ArrayList<Integer> tsp,OriginalGraph G,DataHandler dataHandler) throws IOException{
		
		//0. Initialize all labels:
		
			c[0] = 0.0;
			for(int i=1;i<N;i++){
				c[i] = 999999.0;
				G.getDists()[i-1][i-1] = 0.0;
				G.getwTimes()[i-1][i-1] = 0.0;
				G.getdTimes()[i-1][i-1] = 0.0;
			}
			
		//1. First loop: All split nodes
			
			for(int i=0;i<N-1;i++){ 	//(Tail)
				
				//Check a route where only the current node is visited:
				
					int j = i+1; 		//(Head)
					double[] weights = new double[2];
					weights[0] = 0.0; //cost is 0
					weights[1] = dataHandler.getServicesTimes().get(j-1) + dataHandler.getParkTime(); //time
					
					if(dataHandler.getFixCost()+c[i] < c[j]){
						c[j] = dataHandler.getFixCost() + c[i];
						p[j] = i;
						//Add's the route:
						routeStrings.put((i+";"+j),"-> "+tsp.get(j-1));
						routePrints.put((i+";"+j), new ArrayList<String>());
						routePrints.get((i+";"+j)).add(tsp.get(j-1)+";"+tsp.get(j-1)+"1");
						routeTimes.put((i+";"+j),weights[1]);
						routeCosts.put((i+";"+j),weights[0]+dataHandler.getFixCost());
					}
				
				//The first head:
				
					j = i+2; 		//(Head)
				
				//Creates a pulse handler (an empty board for the pulse):
				
					PulseHandler pH = new PulseHandler(dataHandler,G,tsp);
				
				// A counter to know if the network must be created from scratch or not:
					
					int cont = 1;
					boolean terminar = false;
					
				//2. Second loop: all possible head nodes:
					
					tsp.add(i,N-1);
			
					while(!terminar && j<N){
						
							terminar = true;	
							
							double[] newWeights = new double[2];
						
							if(cont == 1) {
								newWeights = calculateDRWW(i, j+1,pH,dataHandler,G,tsp);
								cont++;
							}else {
								newWeights = calculateDRWWRep(i, j+1,pH,dataHandler,G,tsp);
							}
							
							//Evaluates if the arc is better:
							
							if(newWeights[1] < 999999){
								
								//We must continue:
								
								terminar = false;
								
								if(newWeights[0]+ c[i] < c[j]) {
									
									//Adds the route:
									
									routeStrings.put((i+";"+j),recoverRoute(pH.getPath(),dataHandler,numRoutes,tsp,(i+";"+j)));
									routeTimes.put((i+";"+j),newWeights[1]);
									routeCosts.put((i+";"+j),newWeights[0]);
									
									numRoutes++;
									dataHandler.getRouteStrings().put(numRoutes, routeStrings.get(i+";"+j));
									dataHandler.getRouteTimes().put(numRoutes, routeTimes.get(i+";"+j));
									dataHandler.getRouteCosts().put(numRoutes, routeCosts.get(i+";"+j));
									dataHandler.getRoutePrints().put(numRoutes,routePrints.get(i+";"+j));
									dataHandler.getRouteHeurs().put(numRoutes,this.getHeur());
									this.routeIDs.put((i+"-"+j), numRoutes);
									tsp.remove(i);
									updateClients((i),(j),dataHandler,numRoutes,tsp);
									tsp.add(i,N-1);
									
								//Updates labels:
								
									c[j] = newWeights[0]+ c[i];
									p[j] = i;
								
								
								}
								
							}

							//Update j
							j+=1;
					}
				
					tsp.remove(i);
				
			}

		
	}
	
	/**
	 * This method creates the auxiliary graph for the pulse algorithm and runs the pulse
	 * @param i tail node on the split graph
	 * @param j head node on the split graph
	 * @param pH board for the pulse
	 * @return the weights of the minimum cost path that satisfies the time constraint
	 */
	public double[] calculateDRWW(int i,int j,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer>tspTour) {
		
		//Creates the weights
			
			double[] newWeights = new double[2];
	
		//Create nodes:
			
			for(int l=i+1;l<=j;l++) { //rows
				for(int m=i+1;m<=j;m++) { //columns
					if(l<=m) {
						pH.addNode(l+","+m);
					}
				}
			}
				
		//Modifies the labels of the initial node (Where the shortest path begins)
			
			pH.getNodes().get((i+1)+","+(i+1)).setLabelsCD(G.getDists()[tspTour.get(i)][G.getNumNodes()-1]*dataHandler.getDrivCost(),G.getdTimes()[tspTour.get(i)][G.getNumNodes()-1]); //We set the first move from the CD !!!!!
			
		//Create arcs: (Note that we do it like on the bellman ford procedure !! (Going by layers)
		
			developFirst(i+1,i+1,j,i+1,pH,dataHandler,G,tspTour);
			
			for(int m=i+2;m<j;m++) { //columns
				for(int l=i+1;l<=j;l++) { //rows
					if(l<=m) {
						develop(l,m,j,i+1,pH,dataHandler,G,tspTour);
					}
				}
			}
		
		//This method runs the pulse algorithm
		
			pH.runPulse(i,j,dataHandler.getlT(),dataHandler.getdT(),((i+1)+","+(i+1)),1);
			
		//Saves the solution of the pulse
			
			newWeights[0] = pH.getPrimalBound()+dataHandler.getFixCost();
			newWeights[1] = pH.getTimeStar();

		return newWeights;
	}
	
	/**
	 * This method creates the auxiliary graph for the pulse algorithm and runs the pulse.
	 * However, it only adds a layer of nodes !! Not all of them..
	 * @param i tail node on the split graph
	 * @param j head node on the split graph
	 * @param pH board for the pulse
	 * @return the weights of the minimum cost path that satisfies the time constraint
	 */
	public double[] calculateDRWWRep(int i,int j,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer>tspTour) {
		
		//Creates the weights
		
		double[] newWeights = new double[2];
	
	//Create only nodes that are new !
			
		for(int l=i+1;l<=j;l++) {
			for(int m=j;m<=j;m++) {
				pH.addNode(l+","+m);
			}
		}	
	
	//Modifies the labels of the initial node (Where the shortest path begins) (Maybe this is not needed..)
		
		pH.getNodes().get((i+1)+","+(i+1)).setLabelsCD(G.getDists()[tspTour.get(i)][G.getNumNodes()-1]*dataHandler.getDrivCost(),G.getdTimes()[tspTour.get(i)][G.getNumNodes()-1]); //We set the first move from the CD !!!!!
	
	//Create new arcs:
		
		developFirst2(i+1,i+1,j,i+1,pH,dataHandler,G,tspTour);
		
		for(int m=i+2;m<j;m++) {
			for(int l=i+1;l<j;l++) {
				if(l<=m) {
					develop2(l,m,j,i+1,pH,dataHandler,G,tspTour);
				}
			}
		}
	
	//This method runs the pulse algorithm
		
		pH.runPulse(i,j,dataHandler.getlT(),dataHandler.getdT(),((i+1)+","+(i+1)),1);
		
	//Saves the solution of the pulse
			
		newWeights[0] = pH.getPrimalBound()+dataHandler.getFixCost();
		newWeights[1] = pH.getTimeStar();	


		return newWeights;
	}
	
	/**
	 * This method creates the arc for the pulse board
	 * @param l vertical coordinate - parking spot
	 * @param m horizontal coordinate - client
	 * @param last maximum horizontal coordinate
	 * @param beg initial vertical coordinate
	 */
	public void developFirst(int l,int m,int last,int beg,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer> tspTour){
		
		//Initialize:

			double walkTime = 0;
			double servTime = 0;
			
			servTime+=dataHandler.getServicesTimes().get(tspTour.get(m-1));
			
			int w = tspTour.get(m); //Client!
			int u = w;
			
		//Main loop:
				
			for(int j = m+1;j<=last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v);
				u = v;
					for(int p = beg;p<=last;p++) {
						if(p<=j) {
							
							//Update locations:

								int L = tspTour.get(p-1); 										//Parking spot!
								
								int k2 = tspTour.get(l-1); 										//Parking spot!	

							//Update walking time
							
								double walkTimeA = walkTime+G.getwTimes()[L][w]+G.getwTimes()[v][L]; 	//Movimiento de ps2 a un cliente + De vuelta desde el last client al ps2
								double tourService = servTime;
								if(l==m && l!=p) {
									tourService-=dataHandler.getServicesTimes().get(tspTour.get(m-1));
								}
								
							//Check if we must add the arc (This should go first)
								
							if(walkTimeA+tourService <= dataHandler.getlW()) {
								
								//Parking time:
								
									double parkTime = 0;
									if(l == p) {
										parkTime = dataHandler.getParkTime();
									}else {
										parkTime = dataHandler.getParkTime()*2;
									}
		
								//Driving time and distances	
								
									double divTimeA = G.getdTimes()[k2][L]; 						//Ir del ps1 al ps2
									double divDisA = G.getDists()[k2][L]; 							//Ir del ps1 al ps2

								//Update total time
								
									double totTimeA = walkTimeA+divTimeA+parkTime+servTime;
								
									pH.addArc(l+","+m,p+","+j,divDisA*dataHandler.getDrivCost(),totTimeA,walkTimeA);
									//System.out.println("Arc "+(l+","+m+","+p+","+j)+" - "+divDisA*dataHandler.getDrivCost()+" - "+totTimeA+" - "+walkTimeA);
									PulseNode nodeI = pH.getNodes().get(l+","+m);
									PulseNode nodeF = pH.getNodes().get(p+","+j);
								
									if(totTimeA + nodeI.getMinTime()[1] < nodeF.getMinTime()[1]) {
										nodeF.getMinTime()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinTime()[0];
										nodeF.getMinTime()[1] = totTimeA + nodeI.getMinTime()[1];
										nodeF.getMinTime()[2] = walkTimeA + nodeI.getMinTime()[2];
									}
									if(divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0] < nodeF.getMinCost()[0]) {
										nodeF.getMinCost()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0];
										nodeF.getMinCost()[1] = totTimeA + nodeI.getMinCost()[1];
										nodeF.getMinCost()[2] = walkTimeA + nodeI.getMinCost()[2];
									}
									
									if(walkTimeA + nodeI.getMinDist()[2] < nodeF.getMinDist()[2]) {
										nodeF.getMinDist()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinDist()[0];
										nodeF.getMinDist()[1] = totTimeA + nodeI.getMinDist()[1];
										nodeF.getMinDist()[2] = walkTimeA + nodeI.getMinDist()[2];
									}
								
							}
						}
					}
			}
	}
	
	
	
	/**
	 * This method creates the arc for the pulse board
	 * @param l vertical coordinate - parking spot
	 * @param m horizontal coordinate - client
	 * @param last maximum horizontal coordinate
	 * @param beg initial vertical coordinate
	 */
	public void develop(int l,int m,int last,int beg,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer> tspTour){
		
		//Initialize:

			double walkTime = 0;
			double servTime = 0;
			
			int w = tspTour.get(m); //Client!
			int u = w;
			
		//Main loop:
				
			for(int j = m+1;j<=last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v);
				u = v;
					for(int p = beg;p<=last;p++) {
						if(p<=j) {
							
							//Update locations:

								int L = tspTour.get(p-1); 										//Parking spot!
								
								int k2 = tspTour.get(l-1); 										//Parking spot!	

							//Update walking time
							
								double walkTimeA = walkTime+G.getwTimes()[L][w]+G.getwTimes()[v][L]; 	//Movimiento de ps2 a un cliente + De vuelta desde el last client al ps2
								
								
								double tourService = servTime;
								if(l==m && l!=p) {
									tourService-=dataHandler.getServicesTimes().get(tspTour.get(m-1));
								}
							
							//System.out.println("Considering arc from "+(l+","+m)+" to "+(p+","+j)+" - "+beg+" - "+last);		
							//Check if we must add the arc (This should go first)
								
							
							if(walkTimeA+tourService<=dataHandler.getlW()) {
								
								//Parking time:
								
									double parkTime = 0;
									if(l != p) {
										parkTime = dataHandler.getParkTime();
									}//else {
									//	parkTime = dataHandler.getParkTime()*2;
									//}
		
								//Driving time and distances	
								
									double divTimeA = G.getdTimes()[k2][L]; 						//Ir del ps1 al ps2
									double divDisA = G.getDists()[k2][L]; 							//Ir del ps1 al ps2

								//Update total time
								
									double totTimeA = walkTimeA+divTimeA+parkTime+servTime;
									
									pH.addArc(l+","+m,p+","+j,divDisA*dataHandler.getDrivCost(),totTimeA,walkTimeA);
									PulseNode nodeI = pH.getNodes().get(l+","+m);
									PulseNode nodeF = pH.getNodes().get(p+","+j);
									if(totTimeA + nodeI.getMinTime()[1] < nodeF.getMinTime()[1]) {
										nodeF.getMinTime()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinTime()[0];
										nodeF.getMinTime()[1] = totTimeA + nodeI.getMinTime()[1];
										nodeF.getMinTime()[2] = walkTimeA + nodeI.getMinTime()[2];
									}
									if(divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0] < nodeF.getMinCost()[0]) {
										nodeF.getMinCost()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0];
										nodeF.getMinCost()[1] = totTimeA + nodeI.getMinCost()[1];
										nodeF.getMinCost()[2] = walkTimeA + nodeI.getMinCost()[2];
									}
									
									if(walkTimeA + nodeI.getMinDist()[2] < nodeF.getMinDist()[2]) {
										nodeF.getMinDist()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinDist()[0];
										nodeF.getMinDist()[1] = totTimeA + nodeI.getMinDist()[1];
										nodeF.getMinDist()[2] = walkTimeA + nodeI.getMinDist()[2];
									}
								
							}
						}
					}
			}
	}
	
	/**
	 * This method adds only the new arcs !! This is excelent
	 * @param i vertical coordinate - parking spot
	 * @param k horizontal coordinate - client
	 * @param last maximum horizontal coordinate
	 * @param beg initial vertical coordinate
	 */
	public void develop2(int l,int m,int last,int beg,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer> tspTour){
		
		//Initialize:

			double walkTime = 0;
			double servTime = 0;
			
			int w = tspTour.get(m); //Client!
			int u = w;
			
		//Main loop:
				
			for(int j = m+1;j<last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v) ;
				u = v;
			}
			
			for(int j = last;j<=last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v);
				
				u = v;
			
					for(int p = beg;p<=last;p++) {
						if(p<=j) {
							
							//Update locations:

							int L = tspTour.get(p-1); 										//Parking spot!
							
							int k2 = tspTour.get(l-1); 										//Parking spot!	

						//Update walking time
						
							double walkTimeA = walkTime+G.getwTimes()[L][w]+G.getwTimes()[v][L]; 	//Movimiento de ps2 a un cliente + De vuelta desde el last client al ps2
							double tourService = servTime;
							if(l==m && l!=p) {
								tourService-=dataHandler.getServicesTimes().get(tspTour.get(m-1));
							}
							
						//Check if we must add the arc (This should go first)
						//System.out.println("Considering arc from "+(l+","+m)+" to "+(p+","+j)+" - "+beg+" - "+last+" - "+walkTimeA);	
						if(walkTimeA+tourService<=dataHandler.getlW()) {
							
							if((l == p) || (p == j) ) {
								
							
								//Parking time:
								
									double parkTime = 0;
									if(l != p) {
										parkTime = dataHandler.getParkTime();
									}//else {
									//	parkTime = dataHandler.getParkTime()*2;
									//}
		
								//Driving time and distances	
								
									double divTimeA = G.getdTimes()[k2][L]; 						//Ir del ps1 al ps2
									double divDisA = G.getDists()[k2][L]; 							//Ir del ps1 al ps2
	
								//Update total time
								
									double totTimeA = walkTimeA+divTimeA+parkTime+servTime;
								
									pH.addArc(l+","+m,p+","+j,divDisA*dataHandler.getDrivCost(),totTimeA,walkTimeA);
									PulseNode nodeI = pH.getNodes().get(l+","+m);
									PulseNode nodeF = pH.getNodes().get(p+","+j);
									if(totTimeA + nodeI.getMinTime()[1] < nodeF.getMinTime()[1]) {
										nodeF.getMinTime()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinTime()[0];
										nodeF.getMinTime()[1] = totTimeA + nodeI.getMinTime()[1];
										nodeF.getMinTime()[2] = walkTimeA + nodeI.getMinTime()[2];
									}
									if(divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0] < nodeF.getMinCost()[0]) {
										nodeF.getMinCost()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0];
										nodeF.getMinCost()[1] = totTimeA + nodeI.getMinCost()[1];
										nodeF.getMinCost()[2] = walkTimeA + nodeI.getMinCost()[2];
									}
									
									if(walkTimeA + nodeI.getMinDist()[2] < nodeF.getMinDist()[2]) {
										nodeF.getMinDist()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinDist()[0];
										nodeF.getMinDist()[1] = totTimeA + nodeI.getMinDist()[1];
										nodeF.getMinDist()[2] = walkTimeA + nodeI.getMinDist()[2];
									}
							}
						}
					
					}
				}
			}
	}

	/**
	 * This method adds only the new arcs !! This is excelent
	 * @param i vertical coordinate - parking spot
	 * @param k horizontal coordinate - client
	 * @param last maximum horizontal coordinate
	 * @param beg initial vertical coordinate
	 */
	public void developFirst2(int l,int m,int last,int beg,PulseHandler pH,DataHandler dataHandler,OriginalGraph G,ArrayList<Integer> tspTour){
		
		//Initialize:

			double walkTime = 0;
			double servTime = 0;
			
			servTime+=dataHandler.getServicesTimes().get(tspTour.get(m-1));
			
			
			int w = tspTour.get(m); //Client!
			int u = w;
			
		//Main loop:
				
			for(int j = m+1;j<last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v);
				u = v;
			}
			
			for(int j = last;j<=last;j++) {
				int v = tspTour.get(j-1); 													//Client!
				
				walkTime+=G.getwTimes()[u][v];												//Going forward (between clients)
				servTime+=dataHandler.getServicesTimes().get(v);
				
				u = v;
			
					for(int p = beg;p<=last;p++) {
						if(p<=j) {
							
							//Update locations:

							int L = tspTour.get(p-1); 										//Parking spot!
							
							int k2 = tspTour.get(l-1); 										//Parking spot!	

						//Update walking time
						
							double walkTimeA = walkTime+G.getwTimes()[L][w]+G.getwTimes()[v][L]; 	//Movimiento de ps2 a un cliente + De vuelta desde el last client al ps2
							double tourService = servTime;
							if(l==m && l!=p) {
								tourService-=dataHandler.getServicesTimes().get(tspTour.get(m-1));
							}
							
						//Check if we must add the arc (This should go first)
						
						if(walkTimeA+tourService<=dataHandler.getlW()) {
							
							if((l == p) || (p == j) ) {
								
								//Parking time:
								
									double parkTime = 0;
									if(l == p) {
										parkTime = dataHandler.getParkTime();
									}else {
										parkTime = dataHandler.getParkTime()*2;
									}
		
								//Driving time and distances	
								
									double divTimeA = G.getdTimes()[k2][L]; 						//Ir del ps1 al ps2
									double divDisA = G.getDists()[k2][L]; 							//Ir del ps1 al ps2
	
								//Update total time
								
									double totTimeA = walkTimeA+divTimeA+parkTime+servTime;
								
									pH.addArc(l+","+m,p+","+j,divDisA*dataHandler.getDrivCost(),totTimeA,walkTimeA);
									PulseNode nodeI = pH.getNodes().get(l+","+m);
									PulseNode nodeF = pH.getNodes().get(p+","+j);
									if(totTimeA + nodeI.getMinTime()[1] < nodeF.getMinTime()[1]) {
										nodeF.getMinTime()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinTime()[0];
										nodeF.getMinTime()[1] = totTimeA + nodeI.getMinTime()[1];
										nodeF.getMinTime()[2] = walkTimeA + nodeI.getMinTime()[2];
									}
									if(divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0] < nodeF.getMinCost()[0]) {
										nodeF.getMinCost()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinCost()[0];
										nodeF.getMinCost()[1] = totTimeA + nodeI.getMinCost()[1];
										nodeF.getMinCost()[2] = walkTimeA + nodeI.getMinCost()[2];
									}
									
									if(walkTimeA + nodeI.getMinDist()[2] < nodeF.getMinDist()[2]) {
										nodeF.getMinDist()[0] = divDisA*dataHandler.getDrivCost() + nodeI.getMinDist()[0];
										nodeF.getMinDist()[1] = totTimeA + nodeI.getMinDist()[1];
										nodeF.getMinDist()[2] = walkTimeA + nodeI.getMinDist()[2];
									}
							}
						}
					
					}
				}
			}
	}
	
	/**
	 * This method recovers the solution of the shortest path
	 */
	public void recoverSP(DataHandler dataHandler,ArrayList<Integer> tspTour) {
		int act = N-1;

		while(act!=0){
			int act2 = p[act];
			numRoutes++;
			optimalRoutes.add(numRoutes);
			dataHandler.getRouteStrings().put(numRoutes, routeStrings.get(act2+";"+act));
			dataHandler.getRouteTimes().put(numRoutes, routeTimes.get(act2+";"+act));
			dataHandler.getRouteCosts().put(numRoutes, routeCosts.get(act2+";"+act));
			dataHandler.getRoutePrints().put(numRoutes,routePrints.get(act2+";"+act));
			dataHandler.getRouteHeurs().put(numRoutes,this.getHeur());
			updateClients((act2),(act),dataHandler,numRoutes,tspTour);
			act = act2;
		}
	}
	
	/**
	 * This method recovers the solution of the shortest path
	 */
	public void recoverSP2(DataHandler dataHandler,ArrayList<Integer> tspTour) {
		int act = N-1;

		while(act!=0){
			int act2 = p[act];
			optimalRoutes.add(this.routeIDs.get((act2+"-"+act)));
			act = act2;
		}
	}
	
	/**
	 * This method finds all the clients in a route
	 * @param i
	 * @param j
	 */
	public void updateClients(int i,int j,DataHandler dataHandler,int numRoutes,ArrayList<Integer>tspTour){
		for(int k = i;k<j;k++) {
			dataHandler.getNodeRoutes().get(tspTour.get(k)).add(numRoutes);
		}
	}
	/**
	 * Recovers the final cost
	 * @return
	 */
	public double recoverCost() {
		return c[N-1];
	}
	
	/**
	 * This method recovers the route
	 * @param route
	 * @return
	 */
	public String recoverRoute(ArrayList<String> route,DataHandler dataHandler,int numRoutes,ArrayList<Integer>tspTour,String ID) {
		String rta = "";
		routePrints.put(ID,new ArrayList<String>());
		String[] auxNodeId1 = route.get(route.size()-1).split(",");
		String arc = (-1)+";"+tspTour.get(Integer.parseInt(auxNodeId1[0])-1)+";"+1;
		routePrints.get(ID).add(arc);
		rta+="CD -> ";
		for(int i=route.size()-1;i>=3;i--) {
			String[] nodeId1 = route.get(i).split(",");
			String[] nodeId2 = route.get(i-1).split(",");
			int i1 = Integer.parseInt(nodeId1[0]);
			int j1 = Integer.parseInt(nodeId1[1]);
			int i2 = Integer.parseInt(nodeId2[0]);
			int j2 = Integer.parseInt(nodeId2[1]);
			rta+=calculateRouteAux(i1-1,j1-1,i2-1,j2-1,tspTour)+" || ";
			routePrints(i1-1,j1-1,i2-1,j2-1,tspTour,dataHandler,ID);
		}
		
		String[] nodeId1 = route.get(2).split(",");
		String[] nodeId2 = route.get(1).split(",");
		int i1 = Integer.parseInt(nodeId1[0]);
		int j1 = Integer.parseInt(nodeId1[1]);
		int i2 = Integer.parseInt(nodeId2[0]);
		int j2 = Integer.parseInt(nodeId2[1]);
		arc = tspTour.get(i2-1)+";"+(-1)+";"+1;
		routePrints.get(ID).add(arc);
		rta+=calculateRouteAux(i1-1,j1-1,i2-1,j2-1,tspTour)+" -> CD ";
		routePrints(i1-1,j1-1,i2-1,j2-1,tspTour,dataHandler,ID);
		return rta;
	}
	
	/**
	 * 
	 * @param i1
	 * @param j1
	 * @param i2
	 * @param j2
	 * @return
	 */
	public String calculateRouteAux(int i1, int j1, int i2, int j2,ArrayList<Integer> tspTour) {
		String route = "";
		if(i1 == i2) {
			route+=tspTour.get(i1)+" --- ";
			for(int j = j1+1; j<=j2;j++) {
				if(j>i1) {
					route+=tspTour.get(j)+" --- ";
				}
			}
			route+=tspTour.get(i2);
		}else if(j2 == i2 && j1 + 1 == j2) {
			route+=tspTour.get(i1)+" -> "+tspTour.get(i2);
		}else {
			route+=tspTour.get(i1)+" -> "+tspTour.get(i2);
			for(int j = j1+1; j<=j2;j++) {
				if(j != i2 || (j>j1+1 && j<j2)) {
					route+=" --- "+tspTour.get(j);
				}
			}
			route+=" --- "+tspTour.get(i2);
		}
		return route;
	}
	
	/**
	 * This method allows for an easy printing in R
	 * @param i1
	 * @param j1
	 * @param i2
	 * @param j2
	 */
	public void routePrints(int i1, int j1, int i2, int j2,ArrayList<Integer>tspTour,DataHandler dataHandler,String ID) {
		ArrayList<Integer> routeA = new ArrayList<Integer>();
		int typ = 1;
		if(j2 == i2 && j1 + 1 == j2) {
			routeA.add(tspTour.get(i1));
			routeA.add(tspTour.get(i2));
			typ = 2;
		}else {
			routeA.add(tspTour.get(i1));
			routeA.add(tspTour.get(i2));
			for(int j = j1+1; j<=j2;j++) {
				if(j != i2 || (j>j1+1 && j<j2)) {
					routeA.add(tspTour.get(j));
				}
			}
			routeA.add(tspTour.get(i2));
			typ = 3;
		}
		
		String arc = "";
		if(typ == 1) {
			for(int i = 0;i<routeA.size()-1;i++) {
				arc = routeA.get(i)+";"+routeA.get(i+1)+";"+2;
				routePrints.get(ID).add(arc);
			}
		}else if(typ == 2) {
			arc = routeA.get(0)+";"+routeA.get(1)+";"+1;
			routePrints.get(ID).add(arc);
		}else {
			arc = routeA.get(0)+";"+routeA.get(1)+";"+1;
			routePrints.get(ID).add(arc);
			for(int i = 1;i<routeA.size()-1;i++) {
				arc = routeA.get(i)+";"+routeA.get(i+1)+";"+2;
				routePrints.get(ID).add(arc);
			}
			
		}
	}

	/**
	 * @return the numRoutes
	 */
	public int getNumRoutes() {
		return numRoutes;
	}

	/**
	 * @param numRoutes the numRoutes to set
	 */
	public void setNumRoutes(int numRoutes) {
		this.numRoutes = numRoutes;
	}


	/**
	 * @return the optimalRoutes
	 */
	public ArrayList<Integer> getOptimalRoutes() {
		return optimalRoutes;
	}

	/**
	 * @param optimalRoutes the optimalRoutes to set
	 */
	public void setOptimalRoutes(ArrayList<Integer> optimalRoutes) {
		this.optimalRoutes = optimalRoutes;
	}

	public int getHeur() {
		return heur;
	}

	public void setHeur(int heur) {
		this.heur = heur;
	}
	
}
