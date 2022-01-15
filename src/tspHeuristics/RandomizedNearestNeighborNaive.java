package tspHeuristics;

import java.util.ArrayList;

import dataStructures.DataHandler;
import dataStructures.OriginalArc;
import dataStructures.OriginalGraph;

/**
 * This class implements the RNN heuristic for the tsp
 *
 */

public class RandomizedNearestNeighborNaive {

	/**
	 * The TSP like tour
	 */
	private ArrayList<Integer> tour;
	
	/**
	 * Randomization factor
	 */
	
	private int kk;
	
	/**
	 * The current cost of the tsp tour
	 */
	private double cost;
	
	
	/**
	 * This method creates an instance for the heuristic
	 */
	public RandomizedNearestNeighborNaive(OriginalGraph G,DataHandler data, int fact) {
		kk = fact;
		cost = 0.0;
		this.setTour(new ArrayList<Integer>());
		this.run(G,data);
	}
	
	/**
	 * This method runs the heuristic
	 */
	public void run(OriginalGraph G,DataHandler data) {
		
		//-1. Marca al depot para no tener que tomarlo:
		
			G.getMark()[G.getNumNodes()-1] = true;
		
		//0. Selects a node:
		
			int u = (int) Math.floor(Math.random()*(G.getNumNodes()-2));
			//int u = 0;
			
			getTour().add(u);

			G.getMark()[u] = true;
			
		//1. Main method:
			
			//Stop criteria:
			
			boolean termine = false;
			
			//While all nodes have not been assigned:

			while(!termine) {
				
				//1.1 Searches the nearest neighbor to u
				
					int v = searchNN(G,u);
				
				//1.2 Adds v to the tour
					
					getTour().add(v);
				
				//1.3 Checks the stopping criteria
				
					if(getTour().size() == G.getNumNodes()-1) {
						termine = true;
					}
					
					u = v;
			}

		
	}

	public int searchNN(OriginalGraph G,int u) {
		
		int ran = (int) Math.floor(Math.random()*kk+1);
		ArrayList<OriginalArc> fS = G.getNodes()[u].getForwardStar();
		if(!G.getMark()[fS.get(ran-1).getHead()]){
			G.getMark()[fS.get(ran-1).getHead()] = true;
			return fS.get(ran-1).getHead();
		}
		//System.out.println("Pase con "+u+" - "+ran+" - "+fS.get(ran-1).getHead());
		int busc = 0;
		int may = 0;
		for(int i=0;i<fS.size();i++) {
			if(!G.getMark()[fS.get(i).getHead()]){
				busc+=1;
				may = i;
			}
			if(busc==kk) {
				G.getMark()[fS.get(i).getHead()] = true;
				return fS.get(i).getHead();
			}
		}
		
		if(!G.getMark()[fS.get(may).getHead()]){
			G.getMark()[fS.get(may).getHead()] = true;
			return fS.get(may).getHead();
		}
		for(int i=0;i<fS.size();i++) {
			if(!G.getMark()[fS.get(i).getHead()]){
				G.getMark()[fS.get(i).getHead()] = true;
				return fS.get(i).getHead();
			}
		}
		
		return busc;
	}

	public int evaluateInsertion(int v,DataHandler data,OriginalGraph G) {
		int pos = 0;
		double inc = Double.POSITIVE_INFINITY;
		
		//Evaluate all  the possibilities:
		
		for(int i=1;i<getTour().size();i++) {
			int j = getTour().get(i);
			int k = getTour().get(i-1);
			double auxInc = 0.0;
			if(j != -1 && k!= -1) {
				auxInc = -1*(G.getDists()[k][j]) + G.getDists()[k][v] + G.getDists()[v][j];
			}
			else if(j == -1) {
				auxInc = -1*(G.getDists()[k][G.getNumNodes()-1]) + G.getDists()[k][v] + G.getDists()[v][G.getNumNodes()-1];
			}
			else if(k == -1) {
				auxInc = -1*(G.getDists()[j][G.getNumNodes()-1]) +  G.getDists()[v][j] + G.getDists()[v][G.getNumNodes()-1];
			}
			
			if(auxInc < inc) {
				pos = i;
				inc = auxInc;
			}
		}
		
		updateCost(cost+inc);
		
		return pos;
	}
	
	/**
	 * @return the tour
	 */
	public ArrayList<Integer> getTour() {
		return tour;
	}

	/**
	 * @param tour the tour to set
	 */
	public void setTour(ArrayList<Integer> tour) {
		this.tour = tour;
	}

	/**
	 * @return the kk
	 */
	public int getKk() {
		return kk;
	}

	/**
	 * @param kk the kk to set
	 */
	public void setKk(int kk) {
		this.kk = kk;
	}
	
	/**
	 * Calculate the cost of the first insertion
	 * @param node
	 * @return
	 */
	public double calculateFirstCost(int node,OriginalGraph graph) {
		//The first trip between the depot and the client and The final trip between the client and the depot
		return 2*graph.getDists()[node][graph.getNumNodes()-1];
	}
	
	/**
	 * This method updates the cost of the TSP
	 * @param c
	 */
	public void updateCost(double c) {
		cost = c;
	}
	
}
