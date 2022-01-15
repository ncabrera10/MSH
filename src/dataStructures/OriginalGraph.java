package dataStructures;

import java.util.ArrayList;

/**
 * This class represents the original graph
 *
 */
public class OriginalGraph {

	/**
	 * Number of nodes
	 */
	
	private int numNodes;
	
	/**
	 * Number of arcs
	 */
	
	private int numArcs;
	
	/**
	 * distance matrix
	 */
	private Double[][] Dists;
	
	/**
	 * Driving times matrix
	 */
	private Double[][] dTimes;
	
	/**
	 * Walking times matrix
	 */
	private Double[][] wTimes;
	
	/**
	 * Graph nodes
	 */
	private OriginalNode[] nodes;
	
	/**
	 * List of nodes id ordered by dMin
	 */
	
	private ArrayList<Integer> nodesIdDMin;
		
	/**
	 * List of nodes id ordered by dMax
	 */
		
	private ArrayList<Integer> nodesIdDMax;
		
	/**
	 * List of nodes id ordered by dMax
	 */
		
	private ArrayList<Integer> nodesIdDelta;
	
	/**
	 * Mark down
	 */
	
	private boolean[]mark;
	
	/**
	 * This method creates a graph
	 * @param n number of nodes
	 * @param a number of arcs
	 */
	public OriginalGraph(int n, int a) {
		numNodes = n;
		numArcs = a;
		Dists = new Double[n][n];
		dTimes = new Double[n][n];
		wTimes = new Double[n][n];
		nodes = new OriginalNode[n];
		nodesIdDMin = new ArrayList<Integer>();
		nodesIdDMax = new ArrayList<Integer>();
		nodesIdDelta = new ArrayList<Integer>();
		mark = new boolean[n];
	}
	
	/**
	 * This method creates all nodes
	 * @param n number of nodes
	 */
	public void createNodes(int n){
		for(int i=0;i<n;i++){
			nodes[i] = new OriginalNode(i);
			if(i<n) {
			nodesIdDMin.add(i);
			nodesIdDMax.add(i);
			nodesIdDelta.add(i);
			}
		}
	}

	/**
	 * This method adds an arc
	 * @param t tail
	 * @param h head
	 * @param a distance
	 * @param b driv time
	 * @param d walk time
	 */
	public void addArc(int t, int h, double a, double b, double d) {
		Dists[t][h] = a;
		dTimes[t][h] = b;
		wTimes[t][h] = d;
		nodes[t].addOutGoingArc(new OriginalArc(t,h,a));
	}
	
	/**
	 * @return the numNodes
	 */
	public int getNumNodes() {
		return numNodes;
	}

	/**
	 * @param numNodes the numNodes to set
	 */
	public void setNumNodes(int numNodes) {
		this.numNodes = numNodes;
	}

	/**
	 * @return the numArcs
	 */
	public int getNumArcs() {
		return numArcs;
	}

	/**
	 * @param numArcs the numArcs to set
	 */
	public void setNumArcs(int numArcs) {
		this.numArcs = numArcs;
	}

	/**
	 * @return the Dists
	 */
	public Double[][] getDists() {
		return Dists;
	}


	/**
	 * @return the dTimes
	 */
	public Double[][] getdTimes() {
		return dTimes;
	}

	/**
	 * @param dTimes the dTimes to set
	 */
	public void setdTimes(Double[][] dTimes) {
		this.dTimes = dTimes;
	}

	/**
	 * @return the wTimes
	 */
	public Double[][] getwTimes() {
		return wTimes;
	}

	/**
	 * @param wTimes the wTimes to set
	 */
	public void setwTimes(Double[][] wTimes) {
		this.wTimes = wTimes;
	}

	/**
	 * @return the nodes
	 */
	public OriginalNode[] getNodes() {
		return nodes;
	}

	/**
	 * @param nodes the nodes to set
	 */
	public void setNodes(OriginalNode[] nodes) {
		this.nodes = nodes;
	}

	/**
	 * @return the nodesIdDMin
	 */
	public ArrayList<Integer> getNodesIdDMin() {
		return nodesIdDMin;
	}

	/**
	 * @param nodesIdDMin the nodesIdDMin to set
	 */
	public void setNodesIdDMin(ArrayList<Integer> nodesIdDMin) {
		this.nodesIdDMin = nodesIdDMin;
	}

	/**
	 * @return the nodesIdDMax
	 */
	public ArrayList<Integer> getNodesIdDMax() {
		return nodesIdDMax;
	}

	/**
	 * @param nodesIdDMax the nodesIdDMax to set
	 */
	public void setNodesIdDMax(ArrayList<Integer> nodesIdDMax) {
		this.nodesIdDMax = nodesIdDMax;
	}

	/**
	 * @return the nodesIdDelta
	 */
	public ArrayList<Integer> getNodesIdDelta() {
		return nodesIdDelta;
	}

	/**
	 * @param nodesIdDelta the nodesIdDelta to set
	 */
	public void setNodesIdDelta(ArrayList<Integer> nodesIdDelta) {
		this.nodesIdDelta = nodesIdDelta;
	}

	/**
	 * @return the mark
	 */
	public boolean[] getMark() {
		return mark;
	}

	/**
	 * @param mark the mark to set
	 */
	public void setMark(boolean[] mark) {
		this.mark = mark;
	}
	
	//FOR THE TSP's
	
	/**
	 * This method sorts both lists
	 */
	public void sortNodes() {
		this.sortDMin(nodesIdDMin);
		this.sortDMax(nodesIdDMax);
		//calculateDeltas();
		this.sortDelta(nodesIdDelta);

	}

	//Sorter's

	public void sortDMin(ArrayList<Integer>set) {
		QS(set,0,set.size()-1);
	}
	
	public void QS(ArrayList<Integer>e, int b,int t) {
		int pivote;
	     if(b < t){
	        pivote=colocar(e,b,t);
	        QS(e,b,pivote-1);
	        QS(e,pivote+1,t);
	     }  
	}
	
	public int colocar(ArrayList<Integer>e,int b,int t) {
		int i;
	    int pivote;
	    double valor_pivote;
	    Integer temp;
	    //System.out.println("Encontre  al colocar");
	    pivote = b;
	    valor_pivote = nodes[e.get(pivote)].getDmin();
	    //System.out.println((e.get(pivote))+" - "+valor_pivote);
	    for (i=b+1; i<=t; i++){
	        if (nodes[e.get(i)].getDmin() < valor_pivote){
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

	public void sortDMax(ArrayList<Integer>set) {
		QS2(set,0,set.size()-1);
	}
	
	public void QS2(ArrayList<Integer>e, int b,int t) {
		int pivote;
	     if(b < t){
	        pivote=colocar2(e,b,t);
	        QS2(e,b,pivote-1);
	        QS2(e,pivote+1,t);
	     }  
	}
	
	public int colocar2(ArrayList<Integer>e,int b,int t) {
		int i;
	    int pivote;
	    double valor_pivote;
	    Integer temp;
	    pivote = b;
	    valor_pivote = nodes[e.get(pivote)].getDmax();
	    for (i=b+1; i<=t; i++){
	        if (nodes[e.get(i)].getDmax() < valor_pivote){
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
	
	/**
	 * Sorts the list
	 * @param set
	 */
	public void sortDelta(ArrayList<Integer>set) {
		QS3(set,0,set.size()-1);
	}
	
	/**
	 * Sorts the list
	 * @param e
	 * @param b
	 * @param t
	 */
	public void QS3(ArrayList<Integer>e, int b,int t) {
		int pivote;
	     if(b < t){
	        pivote=colocar3(e,b,t);
	        QS3(e,b,pivote-1);
	        QS3(e,pivote+1,t);
	     }  
	}
	
	/**
	 * This method helps to order a list
	 * @param e
	 * @param b
	 * @param t
	 * @return
	 */
	public int colocar3(ArrayList<Integer>e,int b,int t) {
		int i;
	    int pivote;
	    double valor_pivote;
	    Integer temp;
	    pivote = b;
	    valor_pivote = nodes[e.get(pivote)].getMinDelta();
	    for (i=b+1; i<=t; i++){
	        if (nodes[e.get(i)].getMinDelta() < valor_pivote){
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
	
	/**
	 * This method calculates deltas for each node
	 */
	public void calculateDeltas() {
		
		//Move to every node
		
		for(int i=0;i<numNodes-1;i++) {
			OriginalNode node = nodes[i];
			double currentDelta = node.getMinDelta();
			//Move to every other pair 
			for(int j=0;j<numNodes-1;j++) {
				if(j != i) {
					for(int k=0;k<numNodes-1;k++) {
						if(k != i && k!=j) {
							double candidateDelta = Dists[j][i]+Dists[i][k]-Dists[j][k];
							if(candidateDelta < currentDelta) {
								currentDelta = candidateDelta;
								node.setMinDelta(currentDelta);
							}
						}
					}
				}
			}
			
		}
	}
	
	/**
	 * This method resets the marks
	 */
	public void resetMarks() {
		mark = new boolean[numNodes];
	}
}
