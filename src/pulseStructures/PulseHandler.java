package pulseStructures;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Iterator;

import dataStructures.DataHandler;
import dataStructures.OriginalGraph;

/**
 * This class handles the pulse algorithm.
 * @author nick0
 *
 */
public class PulseHandler {

	/**
	 * Number of nodes
	 */
	
	private int idCount = 0;
	
	/**
	 * List of nodes
	 */
	private Hashtable<String,PulseNode> nodes;
	
	/**
	 * Best solution found so far
	 */
	private double primalBound;
	
	/**
	 * Time associated to the best sol found so far
	 */
	
	private double timeStar;
	
	/**
	 * Dist associated to the best sol found so far
	 */
	
	private double distStar;
	
	/**
	 * The time constraint
	 */
	
	private  int timeLimit;
	
	/**
	 * The walking distance constraint
	 */
	
	private double distLimit;
	/**
	 * End node id
	 */
	
	private String dest;
	
	/**
	 * The final path (Or partial path)
	 */
	private ArrayList<String> Path;
	
	/**
	 * The end node..which in fact is the start node for this backward pulse
	 */
	private String endNode = "s";
	
	/**
	 * The data handler
	 */
	
	private DataHandler dataHandler;
	
	
	/**
	 * The original graph:
	 */
	
	private OriginalGraph graph;
	
	/**
	 * The order
	 */
	private ArrayList<Integer> tspTour;
	
	/**
	 * Depth for the pulse 
	 */
	private int depth = 2;
	
	/**
	 * Pulse Queue
	 */
	private ArrayList<PulsePending> pendingQueue;
	
	/**
	 * Constant that indicates which strategy found the final path (MT = 1 , MC = 2,MD = 3)
	 */
	private int best = 1;
	
	/**
	 * The time of the pulse (Where the path completion occurred)
	 */
	private double finalTime = 0;
	
	/**
	 * The cost of the pulse (Where the path completion occurred)
	 */
	private double finalCost = 0;
	
	/**
	 * The walking distance of the pulse (Where the path completion occurred)
	 */
	private double finalDist = 0;
	
	/**
	 * To stop the pulse fast
	 */
	private boolean ini = false;
	
	/**
	 * To stop the pulse fast (if a  time limit has been exceeded)
	 */
	private boolean stop = false;
	
	/**
	 * Start time for the pulse
	 */
	private Double iniTime;
	
	/**
	 * Creates a new empty pulse handler (This is like creating a board to put on the network)
	 */
	public PulseHandler(DataHandler data,OriginalGraph g,ArrayList<Integer> tsp) {
		nodes = new Hashtable<String,PulseNode>();
		Path = new ArrayList<String>();
		dataHandler = data;
		tspTour = tsp;
		graph = g;
		pendingQueue = new ArrayList<PulsePending>();
		stop = false;
	}
	

	/**
	 * This method adds a node to the network
	 * @param node node
	 * @param id node id
	 */
	public void addNode(String id) {
		nodes.put(id, new PulseNode(id,idCount));
		idCount++;
	}
	
	/**
	 * Adds a pending pulse
	 * @param p
	 */
	public void addPendingPulse(PulsePending p) {
		pendingQueue.add(p);
	}
	/**
	 * This method adds an arc to the network
	 * @param tID tail node id
	 * @param hID head node id
	 * @param cost arc cost
	 * @param time arc time
	 */
	public void addArc(String tID, String hID, double cost, double time, double dist) {
		new PulseArc(nodes.get(tID),nodes.get(hID),cost,time,dist);
	}
	
	/**
	 * @return the nodes
	 */
	public Hashtable<String, PulseNode> getNodes() {
		return nodes;
	}


	/**
	 * @param nodes the nodes to set
	 */
	public void setNodes(Hashtable<String, PulseNode> nodes) {
		this.nodes = nodes;
	}
	
	/**
	 * This method resets the board
	 */
	public void reset() {
		primalBound = 999999;
		timeStar = 999999;
		distStar = 999999;
		best = 1;
		finalTime = 0;
		finalCost = 0;
		finalDist = 0;
		endNode = "s";
		stop = false;
	}
	
	/**
	 * This is the main pulse procedure
	 * @param beg 
	 * @param last
	 * @param timeC time constraint
	 * @param endNode id of the start node (Or the end node for the backwards pulse)
	 */
	public void runPulse(int beg,int last,int timeC,int distC,String endNode,int se) {
		
		//Reset relevant info
		
			reset();
			Path = new ArrayList<String>();
			//System.out.println("Run the pulse");
			
		//Sets the destination node for the pulse
		
			setDest(endNode);
			
		
		//Create an auxiliary node (The node alpha) an his arcs. In addition it sets the minimum time and the minimum cost
		
			PulseNode node = new PulseNode("s",idCount);
			nodes.put("s", node);
			node.setLabels();
			
			if(se == 1) {
				for(int i=beg+1;i<=last;i++) {  
					this.addArc((i+","+last), "s",graph.getDists()[tspTour.get(i-1)][graph.getNumNodes()-1]*dataHandler.getDrivCost(),graph.getdTimes()[tspTour.get(i-1)][graph.getNumNodes()-1],0);
					PulseNode nodeAux = nodes.get((i+","+last));
					//System.out.println(beg+" - "+last+" - "+tspTour.get(i-1)+" - "+dataHandler.getCdInfo()[tspTour.get(i-1)][0]+" - "+dataHandler.getCdInfo()[tspTour.get(i-1)][1]);
					//Aca faltaria anadir el ultimo viaje para retornar del CD
					if(nodeAux.getMinTime()[1] + graph.getdTimes()[tspTour.get(i-1)][graph.getNumNodes()-1]< node.getMinTime()[1]) {
						node.getMinTime()[1] = nodeAux.getMinTime()[1]+ graph.getdTimes()[tspTour.get(i-1)][graph.getNumNodes()-1];
						node.getMinTime()[0] = nodeAux.getMinTime()[0]+ graph.getDists()[tspTour.get(i-1)][graph.getNumNodes()-1]*dataHandler.getDrivCost();
						node.getMinTime()[2] = nodeAux.getMinTime()[2];
					}
					if(nodeAux.getMinDist()[2] < node.getMinDist()[2]) {
						node.getMinDist()[1] = nodeAux.getMinDist()[1]+ graph.getdTimes()[tspTour.get(i-1)][graph.getNumNodes()-1];
						node.getMinDist()[0] = nodeAux.getMinDist()[0]+ graph.getDists()[tspTour.get(i-1)][graph.getNumNodes()-1]*dataHandler.getDrivCost();
						node.getMinDist()[2] = nodeAux.getMinDist()[2];
					}
					if(nodeAux.getMinCost()[0]+ graph.getDists()[tspTour.get(i-1)][graph.getNumNodes()-1]*dataHandler.getDrivCost() < node.getMinCost()[0]) {
						node.getMinCost()[1] = nodeAux.getMinCost()[1]+ graph.getdTimes()[tspTour.get(i-1)][graph.getNumNodes()-1];
						node.getMinCost()[0] = nodeAux.getMinCost()[0]+ graph.getDists()[tspTour.get(i-1)][graph.getNumNodes()-1]*dataHandler.getDrivCost();
						node.getMinCost()[2] = nodeAux.getMinCost()[2];
					}
				}
			}else {
				for(int i=beg+1;i<=last;i++) {  
					this.addArc((i+","+last), "s",0,0,0);
					PulseNode nodeAux = nodes.get((i+","+last));
					if(nodeAux.getMinTime()[1] < node.getMinTime()[1]) {
						node.getMinTime()[2] = nodeAux.getMinTime()[2];
						node.getMinTime()[1] = nodeAux.getMinTime()[1];
						node.getMinTime()[0] = nodeAux.getMinTime()[0];
					}
					if(nodeAux.getMinDist()[2] < node.getMinDist()[2]) {
						node.getMinDist()[2] = nodeAux.getMinDist()[2];
						node.getMinDist()[1] = nodeAux.getMinDist()[1];
						node.getMinDist()[0] = nodeAux.getMinDist()[0];
					}
					if(nodeAux.getMinCost()[0] < node.getMinCost()[0]) {
						node.getMinCost()[2] = nodeAux.getMinCost()[2];
						node.getMinCost()[1] = nodeAux.getMinCost()[1];
						node.getMinCost()[0] = nodeAux.getMinCost()[0];
					}
				}
			}
			
		
		//Reset pending pulses. This is relevant because of the strategy we have..where we don't create the network from scratch
			
			for(Iterator<PulseNode> iter = nodes.values().iterator();iter.hasNext();) {
				PulseNode n = iter.next();
				n.firstTime = true;
				n.pend = new ArrayList<PulsePending>();
				n.pendT = new ArrayList<PulsePendingT>();
			}
			
		//Set the time constraint 
			
			setTimeLimit(timeC);
			setDistLimit(distC);
			
		//Start the clock (Only for checking)
			
			//Double ITime = (double)System.nanoTime();
			
			if(node.getMinTime()[1] > timeC || node.getMinDist()[2] > distC) { //Check if is infeasible by any of the resources
					
				//Set the primal bound and the time star to a large number
					
					this.setPrimalBound(999999);
					this.setTimeStar(999999);
					this.setDistStar(999999);
						
			}else if(node.getMinCost()[1] <= timeC && node.getMinCost()[2] <= distC) {
				
				//Set the primal bound and the time star
				
					this.setPrimalBound(node.getMinCost()[0]);
					this.timeStar = node.getMinCost()[1];
					this.distStar = node.getMinCost()[2];
					best = 2;
					ini = true;
					this.recoverThePathQ(beg);
					
			}
			else {
					
				//Set the primal bound and the time star with the path of minimum time
					
					this.setPrimalBound(999999);
					this.setTimeStar(999999);
					this.setDistStar(999999);
				
					//Check the minimum cost path:
					
						
					//Check the path of minimum time:
				
						if(node.getMinTime()[2] <= distC & node.getMinTime()[0] < this.getPrimalBound()) {
							
							this.setPrimalBound(node.getMinTime()[0]);
							this.timeStar = node.getMinTime()[1];
							this.distStar = node.getMinTime()[2];
							ini = true;
							best = 1;
						}
						
					// Check the path of minimum walking distance:
						
						if(node.getMinDist()[1] <= timeC & node.getMinDist()[0] < this.getPrimalBound()) {
							
							this.setPrimalBound(node.getMinDist()[0]);
							this.timeStar = node.getMinDist()[1];
							this.distStar = node.getMinDist()[2];
							ini = true;
							best = 3;

						}
						

				//Initial pulse weights
						
					double[] pulseWeights = new double[3];
					pulseWeights[0] = 0;
					pulseWeights[1] = 0;
					pulseWeights[2] = 0;
							
				//Send the first pulse
					
					iniTime = (double) System.nanoTime();
					this.setStop(false);
					
					node.pulseWithQueues(pulseWeights, 0,"s",this);
					
					//Store the number of pulses that are in the queue
					
					int pendingPulses = pendingQueue.size();
				
				//While we have pending pulses we must explore them
					
					while(pendingPulses > 0 && !stop) {
						
						//Recovers the last pulse (and removes it):
        				
        					PulsePending p = pendingQueue.remove(pendingPulses-1);
        					p.setNotTreated(false);
        				
        				//The pendingPulse weights:
        				
        					pulseWeights[0] = p.getCost();
	        				pulseWeights[1] = p.getTime();
	        				pulseWeights[2] = p.getDist();
	        				
	        			//Resumes the pulse
	        				
	        				if(nodes.get(p.getNodeID()).getMinCost()[0] + pulseWeights[0] < primalBound) {
	        					nodes.get(p.getNodeID()).pulseWithQueues(pulseWeights, 0,p.getPredId(),this);
	        				}
	        				
	        			//Updates the global queue size (How many are left)
	        	
	        				pendingPulses = pendingQueue.size();
					}
					
				//Recover the path

					this.recoverThePathQ(beg);	
					
				}
				
		//Remove the auxiliary node
			
			nodes.remove("s");
	}

	/**
	 * This method recovers the final path
	 * @param beg
	 * @return
	 */
	public void recoverThePathQ(int beg){
		
		//First adds the back part of the path
		
			ArrayList<String> path = new ArrayList<String>();
		
			
			if(!ini) {

				path = recoverBackPart(beg);

			}
			
			
			
		//Check if was a MC or a MT path completion
			
			if(best == 1) {
				
				//Completes the path with the MT path
				
					ArrayList<String> auxPath = recoverFrontMT(beg);

					for(int i=0;i<auxPath.size();i++) {
						path.add(auxPath.get(i));
					}
			}else if(best == 2){
				
				//Completes the path with the MC path
					ArrayList<String> auxPath = recoverFrontMC(beg);
					for(int i=0;i<auxPath.size();i++) {
						path.add(auxPath.get(i));
					}
			}
			else if(best == 3){
				
				//Completes the path with the MC path
					ArrayList<String> auxPath = recoverFrontMD(beg);
					for(int i=0;i<auxPath.size();i++) {
						path.add(auxPath.get(i));
					}
			}
		
		//Stores the final path

			for(Iterator<String> iter = path.iterator();iter.hasNext();) {
				Path.add(iter.next());
			}

	}
	
	
	/**
	 * Recovers the final part of the path if it was a MT path completion
	 * @param beg
	 * @return
	 */
	public ArrayList<String> recoverFrontMT(int beg){

		ArrayList<String> path = new ArrayList<String>();
		boolean termine = false;
		String idAct = endNode;
		String idAnt = "";
		path.add(endNode);
		
		while(termine == false) {
			PulseNode actNode = nodes.get(idAct);
			boolean alt = false;
			for(Iterator<PulseArc> iter = actNode.getIncomingArcs().iterator();iter.hasNext() && alt == false;) {
				PulseArc arc = iter.next();
				if(arc.getDist() + finalDist + arc.getTail().getMinTime()[2] - distStar <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) { 
					if(arc.getTime() + finalTime + arc.getTail().getMinTime()[1] - timeStar <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)){
						if(arc.getCost() + finalCost + arc.getTail().getMinTime()[0] - primalBound<  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
							idAct = arc.getTail().getId();
							finalTime += arc.getTime();
							finalCost += arc.getCost();
							finalDist += arc.getDist();
							alt = true;
						}
					}
				}
			}
			
			if(idAct.equals(idAnt)) {
				System.out.println("Trouble in paradise MT "+idAct+" - "+idAnt+" - "+((beg+1)+","+(beg+1))+" - "+(primalBound+375)+" - "+timeStar+" - "+distStar+" - "+finalCost+" - "+finalTime+" - "+finalDist);
				termine = true;
				modifyValues();
			}
			idAnt = idAct;				
			
			path.add(idAct);
			if(idAct.equals((beg+1)+","+(beg+1))) {
				termine = true;

			}
			
		}
		return path;
	}
	
	/**
	 * Recovers the final part of the path if it was a MD path completion
	 * @param beg
	 * @return
	 */
	public ArrayList<String> recoverFrontMD(int beg){

		ArrayList<String> path = new ArrayList<String>();
		boolean termine = false;
		String idAct = endNode;
		String idAnt = "";
		path.add(endNode);
		
		while(termine == false) {
			PulseNode actNode = nodes.get(idAct);
			boolean alt = false;
			for(Iterator<PulseArc> iter = actNode.getIncomingArcs().iterator();iter.hasNext() && alt == false;) {
				PulseArc arc = iter.next();
				if(arc.getDist() + finalDist + arc.getTail().getMinDist()[2] - distStar<  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
					if(arc.getTime() + finalTime + arc.getTail().getMinDist()[1] - timeStar<  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
						if(arc.getCost() + finalCost + arc.getTail().getMinDist()[0]- primalBound<  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
							idAct = arc.getTail().getId();
							finalTime += arc.getTime();
							finalCost += arc.getCost();
							finalDist += arc.getDist();
							alt = true;
						}
					}
				}
			}
			
			if(idAct.equals(idAnt)) {
				System.out.println("Trouble in paradise MT "+idAct+" - "+idAnt+" - "+((beg+1)+","+(beg+1))+" - "+(primalBound+375)+" - "+timeStar+" - "+distStar+" - "+finalCost+" - "+finalTime+" - "+finalDist);
				termine = true;
				modifyValues();
			}
			idAnt = idAct;				
			
			path.add(idAct);
			if(idAct.equals((beg+1)+","+(beg+1))) {
				termine = true;

			}
			
		}
		return path;
	}
	
	/**
	 * Recovers the final part of the path if it was a MC path completion
	 * @param beg
	 * @return
	 */
	public ArrayList<String> recoverFrontMC(int beg){
		ArrayList<String> path = new ArrayList<String>();
		boolean termine = false;
		String idAct = endNode;
		String idAnt = "";
		path.add(endNode);

		while(termine == false) {
			PulseNode actNode = nodes.get(idAct);
			//System.out.println(idAct);
			boolean alt = false;
			for(Iterator<PulseArc> iter = actNode.getIncomingArcs().iterator();iter.hasNext() && alt == false;) {
				PulseArc arc = iter.next();
				if(arc.getDist() + finalDist + arc.getTail().getMinCost()[2] - distStar <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)){
					if (arc.getTime() + finalTime + arc.getTail().getMinCost()[1] - timeStar <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
						if(arc.getCost() + finalCost + arc.getTail().getMinCost()[0]- primalBound <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
							idAct = arc.getTail().getId();
							finalTime += arc.getTime();
							finalCost += arc.getCost();
							finalDist += arc.getDist();
							alt = true;
						}
					}
				}
			}
			if(idAct.equals(idAnt)) {
				System.out.println("Trouble in paradise MC "+idAct+" - "+idAnt+" - "+((beg+1)+","+(beg+1))+" - "+(primalBound+375)+" - "+timeStar+" - "+distStar+" - "+finalCost+" - "+finalTime+" - "+finalDist);
				termine = true;
				modifyValues();
			}
			idAnt = idAct;		
			path.add(idAct);
			if(idAct.equals((beg+1)+","+(beg+1))) {
				termine = true;

			}
		}
		return path;
	}
	
	/**
	 * Go back using the predecessors
	 * @param beg
	 * @return
	 */
	public ArrayList<String> recoverBackPart(int beg){
		ArrayList<String> path = new ArrayList<String>();
		boolean termine = false;
		String nodoActual = "";
		String nodoInicial = endNode;
		
		double costoAcumulado = primalBound - finalCost;
		double tiempoAcumulado = timeStar - finalTime;
		double distAcumulado = distStar - finalDist;
		
		while(termine == false) {
			nodoActual = "s";
			boolean cambie = false;
			ArrayList<PulsePendingT> pendingPulses = nodes.get(nodoInicial).getPendT();
			for(int i = 0; i < pendingPulses.size() && !cambie;i++) {
				PulsePendingT p = pendingPulses.get(i);

				if(Math.abs(p.getCost() + costoAcumulado - primalBound) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION) ) {
					if(Math.abs(p.getTime() + tiempoAcumulado - timeStar) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
						if(Math.abs(p.getDist() + distAcumulado - distStar) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
							nodoActual = p.getPredId();
							PulseNode actNode = nodes.get(nodoInicial);
							for(Iterator<PulseArc> iter = actNode.getOutgoingArcs().iterator();iter.hasNext() && !cambie;) {
								PulseArc arc = iter.next();
								if(arc.getHead().getId().equals(nodoActual)) {
									ArrayList<PulsePendingT> pendingPulsesAux = nodes.get(nodoActual).getPendT();
									for(int ii = 0; ii < pendingPulsesAux.size();ii++) {
										PulsePendingT pp = pendingPulsesAux.get(ii);
										if(Math.abs(pp.getCost() + costoAcumulado + arc.getCost() - primalBound) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
											if(Math.abs(pp.getTime() + tiempoAcumulado + arc.getTime()  - timeStar) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
												if(Math.abs(pp.getDist() + distAcumulado + arc.getDist() - distStar) <  Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
													
												cambie = true;
												costoAcumulado+=arc.getCost();
												tiempoAcumulado+=arc.getTime();
												distAcumulado+=arc.getDist();
												
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
	
			path.add(nodoActual);
			if(nodoActual.equals("s")) {
				termine = true;
				PulseNode actNode = nodes.get(nodoInicial);
				for(Iterator<PulseArc> iter = actNode.getOutgoingArcs().iterator();iter.hasNext();) {
					PulseArc arc = iter.next();
					if(arc.getHead().getId().equals(nodoActual)) {
						costoAcumulado+=arc.getCost();
						tiempoAcumulado+=arc.getTime();
						distAcumulado+=arc.getDist();
					}
				}
				
			}else {
				nodoInicial = nodoActual;
			}
			
		}
	
		
		for (int i = 0; i < path.size() / 2; i++) {
		     Object temp = path.get(i);
		     path.set(i, path.get(path.size() - i - 1));
		     path.set(path.size() - i - 1,  (String) temp);
		 }

		if(Math.abs(costoAcumulado - primalBound) > Math.pow(10,-globalParameters.GlobalParameters.PRECISION) || Math.abs(tiempoAcumulado - timeStar) > Math.pow(10,-globalParameters.GlobalParameters.PRECISION)|| Math.abs(distAcumulado - distStar)> Math.pow(10,-globalParameters.GlobalParameters.PRECISION)) {
			System.out.println("Trouble in paradise back part- "+((beg+1)+","+(beg+1))+" - "+(primalBound+375)+" - "+timeStar+" - "+distStar+" - "+finalCost+" - "+finalTime+" - "+finalDist);
			termine = true;
			modifyValues();
		}
		return path;
	}
	
	
	/**
	 * This method performs a binary search on the penging queue
	 * @param p
	 * @param labels
	 * @return
	 */
	public int binarySearch(PulsePending p, ArrayList<PulsePending> labels) {
		double cScore = p.getSortCriteria();
		boolean cond = true;
		int l = 0; //izq
		int r = labels.size()-1; //der
		int m = (int) ((l + r) / 2); //medio
		double mVal = 0;
		if(labels.size() == 1){
			return 0;
		}else{
			mVal = labels.get(m).getSortCriteria();
		}
		while (cond) {
			//			 System.out.println("murio");
			if (r - l > 1) {
				if (cScore > mVal) {
					r = m;
					m = (int) ((l + r) / 2);
				} else if (cScore < mVal) {
					l = m;
					m = (int) ((l + r) / 2);
				} else if (p.getAuxID()>labels.get(m).getAuxID()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getAuxID()<labels.get(m).getAuxID()){
					l = m;
					m = (int) ((l + r) / 2);
				} else if (p.getTime()>labels.get(m).getTime()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getTime()<labels.get(m).getTime()){
					l = m;
					m = (int) ((l + r) / 2);
				}else if (p.getDist()>labels.get(m).getDist()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getDist()<labels.get(m).getDist()){
					l = m;
					m = (int) ((l + r) / 2);
				}else {
					return m;
				}
				mVal = labels.get(m).getSortCriteria();
			} else {
				cond = false;
				
				if (p.equals(labels.get(r))){
					return r;
				}else if (p.equals(labels.get(l))){
					return l;
				}

			}
		}
		return -1;

	}
	
	/**
	 * This method a pending pulse in order for the queue
	 * @param p
	 * @param labels
	 */
	
	public void addPendingPulse_DOrder(PulsePending p, ArrayList<PulsePending>labels){

		double cScore = p.getSortCriteria();
		boolean cond = true;
		int l = 0; //Por izquierda
		int r = labels.size(); //Por derecha
		int m = (int) ((l + r) / 2); //La mitad
		double mVal = 0;
		if(labels.size() == 0) {
			labels.add(p);
			return;
		}
		else if(labels.size()  == 1) {
			mVal = labels.get(m).getSortCriteria();
			if(cScore == mVal) {
				if(p.getNodeID() == labels.get(m).getNodeID()) {
					labels.add(p.getTime()>labels.get(m).getTime()?0:1,p);
				}else {
					labels.add(p.getAuxID()>labels.get(m).getAuxID()?0:1,p);
				}
			}else {
				labels.add(cScore>mVal?0:1,p);
			}
		}
		else {
			mVal = labels.get(m).getSortCriteria();
		}
		while(cond) {
			if (r - l > 1) {
				if (cScore > mVal) {
					r = m;
					m = (int) ((l + r) / 2);
				} else if (cScore < mVal) {
					l = m;
					m = (int) ((l + r) / 2);
				} else if (p.getAuxID()>labels.get(m).getAuxID()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getAuxID()<labels.get(m).getAuxID()){
					l = m;
					m = (int) ((l + r) / 2);
				}  else if (p.getTime()>labels.get(m).getTime()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getTime()<labels.get(m).getTime()){
					l = m;
					m = (int) ((l + r) / 2);
				}  else if (p.getDist()>labels.get(m).getDist()){
					r = m;
					m = (int) ((l + r) / 2);
				} else if (p.getDist()<labels.get(m).getDist()){
					l = m;
					m = (int) ((l + r) / 2);
				} else {
					labels.add(m, p);
					return;
				}
				mVal = labels.get(m).getSortCriteria();
			} else {
				cond = false;
				if(l == m ){
					if (cScore == mVal){
						if(p.getAuxID()==labels.get(m).getAuxID()){
							labels.add(p.getTime()>labels.get(m).getTime()?l:l+1,p);
						}else{
							labels.add(p.getAuxID()>labels.get(m).getAuxID()?l:l+1,p);
						}							
					}else{
						labels.add(cScore>mVal?l:l+1,p);
					}
				}else if (r == m){
					if (cScore == mVal){
						if(p.getNodeID()==labels.get(m).getNodeID()){
							labels.add(p.getTime()>labels.get(m).getTime()?r:Math.min(r+1, labels.size()),p);
						}else{
							labels.add(p.getAuxID()>labels.get(m).getAuxID()?r:Math.min(r+1, labels.size()),p);
						}
					}else{
						labels.add(cScore>mVal?r:Math.min(r+1, labels.size()),p);
					}
				}else
				{
					System.err.println("LABEL, addLabel ");
				}
				return;
			}
			
			
		}
		
	}

	
	/**
	 * This is a method for the JIC
	 */
	public void modifyValues() {
		setPrimalBound(999999);
		setTimeStar(999999);
		setDistStar(999999);
	}
	
	/**
	 * Returns the destination
	 * @return
	 */
	public String getDest() {
		return dest;
	}

	/**
	 * Sets the final destination
	 * @param dest
	 */
	public void setDest(String dest) {
		this.dest = dest;
	}

	
	
	/**
	 * Returns the time constraint
	 * @return
	 */
	public int getTimeLimit() {
		return timeLimit;
	}

	/**
	 * Sets the time constraint
	 * @param timeLimit
	 */
	public void setTimeLimit(int timeLimit) {
		this.timeLimit = timeLimit;
	}

	/**
	 * Returns the primal bound (Best cost found)
	 * @return
	 */
	public double getPrimalBound() {
		return primalBound;
	}

	/**
	 * This method modifies the primal bound
	 * @param pB
	 */
	public void setPrimalBound(double pB) {
		primalBound = pB;
	}

	/**
	 * This method returns the time(Of the best path found)
	 * @return
	 */
	public double getTimeStar() {
		return timeStar;
	}

	/**
	 * This method sets the time(Of the best path found)
	 */
	public void setTimeStar(double d) {
		timeStar = d;
		
	}
	
	
	public void setDistStar(double d) {
		// TODO Auto-generated method stub
		distStar = d;
	}
	
	/**
	 * @return the distLimit
	 */
	public double getDistLimit() {
		return distLimit;
	}

	/**
	 * @param distLimit the distLimit to set
	 */
	public void setDistLimit(int distLimi) {
		distLimit = distLimi;
	}


	/**
	 * @return the idCount
	 */
	public int getIdCount() {
		return idCount;
	}


	/**
	 * @param idCount the idCount to set
	 */
	public void setIdCount(int idCount) {
		this.idCount = idCount;
	}


	/**
	 * @return the path
	 */
	public ArrayList<String> getPath() {
		return Path;
	}


	/**
	 * @param path the path to set
	 */
	public void setPath(ArrayList<String> path) {
		Path = path;
	}


	/**
	 * @return the endNode
	 */
	public String getEndNode() {
		return endNode;
	}


	/**
	 * @param endNode the endNode to set
	 */
	public void setEndNode(String endNode) {
		this.endNode = endNode;
	}


	/**
	 * @return the dataHandler
	 */
	public DataHandler getDataHandler() {
		return dataHandler;
	}


	/**
	 * @param dataHandler the dataHandler to set
	 */
	public void setDataHandler(DataHandler dataHandler) {
		this.dataHandler = dataHandler;
	}


	/**
	 * @return the graph
	 */
	public OriginalGraph getGraph() {
		return graph;
	}


	/**
	 * @param graph the graph to set
	 */
	public void setGraph(OriginalGraph graph) {
		this.graph = graph;
	}


	/**
	 * @return the tspTour
	 */
	public ArrayList<Integer> getTspTour() {
		return tspTour;
	}


	/**
	 * @param tspTour the tspTour to set
	 */
	public void setTspTour(ArrayList<Integer> tspTour) {
		this.tspTour = tspTour;
	}


	/**
	 * @return the distStar
	 */
	public double getDistStar() {
		return distStar;
	}
	
	
	public int getDepth() {
		return depth;
	}


	/**
	 * @return the pendingQueue
	 */
	public ArrayList<PulsePending> getPendingQueue() {
		return pendingQueue;
	}


	/**
	 * @param pendingQueue the pendingQueue to set
	 */
	public void setPendingQueue(ArrayList<PulsePending> pendingQueue) {
		this.pendingQueue = pendingQueue;
	}


	/**
	 * @return the best
	 */
	public int getBest() {
		return best;
	}


	/**
	 * @param best the best to set
	 */
	public void setBest(int best) {
		this.best = best;
	}


	/**
	 * @return the finalTime
	 */
	public double getFinalTime() {
		return finalTime;
	}


	/**
	 * @param finalTime the finalTime to set
	 */
	public void setFinalTime(double finalTime) {
		this.finalTime = finalTime;
	}


	/**
	 * @return the finalCost
	 */
	public double getFinalCost() {
		return finalCost;
	}


	/**
	 * @param finalCost the finalCost to set
	 */
	public void setFinalCost(double finalCost) {
		this.finalCost = finalCost;
	}


	/**
	 * @return the finalDist
	 */
	public double getFinalDist() {
		return finalDist;
	}


	/**
	 * @param finalDist the finalDist to set
	 */
	public void setFinalDist(double finalDist) {
		this.finalDist = finalDist;
	}


	/**
	 * @param depth the depth to set
	 */
	public void setDepth(int depth) {
		this.depth = depth;
	}


	/**
	 * @return the ini
	 */
	public boolean isIni() {
		return ini;
	}


	/**
	 * @param ini the ini to set
	 */
	public void setIni(boolean ini) {
		this.ini = ini;
	}


	/**
	 * @return the stop
	 */
	public boolean isStop() {
		return stop;
	}


	/**
	 * @param stop the stop to set
	 */
	public void setStop(boolean stop) {
		this.stop = stop;
	}


	/**
	 * @return the iniTime
	 */
	public Double getIniTime() {
		return iniTime;
	}


	/**
	 * @param iniTime the iniTime to set
	 */
	public void setIniTime(Double iniTime) {
		this.iniTime = iniTime;
	}

	
	
	
}
