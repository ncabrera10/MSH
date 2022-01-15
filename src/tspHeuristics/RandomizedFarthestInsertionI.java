package tspHeuristics;

import java.util.ArrayList;
import java.util.Iterator;

import dataStructures.DataHandler;
import dataStructures.OriginalGraph;

public class RandomizedFarthestInsertionI {

	/**
	 * The TSP like tour
	 */
	private ArrayList<Integer> tour;
	
	/**
	 * The current cost of the tsp tour
	 */
	private double cost;
	
	/**
	 * Randomization factor
	 */
	
	private int kk;
	
	/**
	 * List of nodes (That have not been removed)
	 */
	
	private ArrayList<Integer> nodesList1;
	
	/**
	 * List of nodes (That have been removed)
	 */
	
	private ArrayList<Integer> nodesList2;
	
	/**
	 * List of metrics
	 */
	
	private static double[] dMins;
	
	/**
	 * This method creates an instance for the heuristic
	 */
	public RandomizedFarthestInsertionI(OriginalGraph G,DataHandler data,int fact) {
		kk = fact;
		cost = 0.0;
		dMins = new double[G.getNumNodes()];
		nodesList1 = new ArrayList<Integer>();
		for(int i=0;i<G.getNumNodes()-1;i++) {
			nodesList1.add(i);
		}
		nodesList2 = new ArrayList<Integer>();
		setTour(new ArrayList<Integer>());
		run(G,data);
	}
	
	/**
	 * This method runs the heuristic
	 */
	public void run(OriginalGraph G,DataHandler data) {
		
		//-1. Marks the depot:
		
			G.getMark()[G.getNumNodes()-1] = true;
		
		//0. Selects a node:
		
			int ran = (int) Math.floor(Math.random()*(G.getNumNodes()-2));
			int u = nodesList1.get(ran);
			
			//Adds the CD, the node and the CD:
			
				getTour().add(-1);
				getTour().add(u);
				getTour().add(-1);
				updateCost(calculateFirstCost(u,G));
				nodesList1.remove(ran);
					
			//Marks the node:
				
			G.getMark()[u] = true;
			nodesList2.add(u);
			updateDMin(G);
			
		//1. Main method:
			
			//Stop criteria:
			
			boolean termine = false;
			
			//While all nodes have not been assigned:
			
			while(!termine) {
				
				//1.1 Searches the next node to u
				
					int v = searchNI(G,u);
					nodesList2.add(v);
					updateDMin(G);
					
				//1.2 Evaluates in which position should be inserted
					
					int pos = evaluateInsertion(v,data,G);
					
				//1.2 Adds v to the tour
				
					getTour().add(pos,v);
					
				
				//1.3 Checks the stopping criteria
				
					if(getTour().size() == (G.getNumNodes()+2-1)) {
						termine = true;
					}
					
					u = v;
			}
			
		//2. Set's the final tour
			
			getTour().remove(0);
			getTour().remove(getTour().size()-1);
			//System.out.println(nodesList.toString());
			//System.out.println(getTour().toString());
	}

	public int searchNI(OriginalGraph G,int u) {
		int aleat = kk;
		if(nodesList1.size()<kk) {
			aleat = nodesList1.size();
		}
		int ran = (int) Math.floor(Math.random()*aleat+1);
		
		//Check if there are enough nodes:
		
		int busc = 0;
		if(nodesList1.size() >= ran) {
			busc = nodesList1.get(ran-1);
			nodesList1.remove(ran-1);
			return busc;
		}else {
			busc = nodesList1.get(nodesList1.size()-1);
			nodesList1.remove(nodesList1.size()-1);
			return busc;
		}
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
	 * Returns the tsp tour
	 * @return
	 */
	public  ArrayList<Integer> getTour() {
		return tour;
	}

	/**
	 * Sets the final tsp tour
	 * @param tour
	 */
	public void setTour(ArrayList<Integer> tou) {
		tour = tou;
	}
	
	/**
	 * This method updates the cost of the TSP
	 * @param c
	 */
	public void updateCost(double c) {
		cost = c;
	}
	
	/**
	 * Calculate the cost of the first insertion
	 * @param node
	 * @return
	 */
	public double calculateFirstCost(int node,OriginalGraph graph) {
		//The first trip between the depot and the client and The final trip betwen the client and the depot
		return 2*graph.getDists()[node][graph.getNumNodes()-1];
	}
	
	public void updateDMin(OriginalGraph graph) {
		for(Iterator<Integer> iter = nodesList1.iterator();iter.hasNext();) {
			int actNode = iter.next();
			dMins[actNode] = 0;
			for(Iterator<Integer> iter2 = nodesList2.iterator();iter2.hasNext();) {
				int actNode2 = iter2.next();
				if(actNode!=actNode2) {
					if(graph.getDists()[actNode2][actNode] > dMins[actNode]) {
						dMins[actNode] = graph.getDists()[actNode2][actNode];
					}
				}
			}
		}
		sortDMin(nodesList1);
		//System.out.println(nodesList1);
	}
	
	
	public static void sortDMin(ArrayList<Integer>set) {
		QS(set,0,set.size()-1);
	}
	
	public static void QS(ArrayList<Integer>e, int b,int t) {
		int pivote;
	     if(b < t){
	        pivote=colocar(e,b,t);
	        QS(e,b,pivote-1);
	        QS(e,pivote+1,t);
	     }  
	}
	
	public static int colocar(ArrayList<Integer>e,int b,int t) {
		int i;
	    int pivote;
	    double valor_pivote;
	    Integer temp;
	    //System.out.println("Encontre  al colocar");
	    pivote = b;
	    valor_pivote = dMins[e.get(pivote)];
	    //System.out.println((e.get(pivote))+" - "+valor_pivote);
	    for (i=b+1; i<=t; i++){
	        if (dMins[e.get(i)] > valor_pivote){
	                pivote++;    
	                temp= e.get(i);
	                e.set(i, e.get(pivote));
	                e.set(pivote, temp);
	        }
	    }
	    temp=e.get(b);
	    e.set(b, e.get(pivote));
        e.set(pivote, temp);
	    return pivote;
	}
	
	
}
