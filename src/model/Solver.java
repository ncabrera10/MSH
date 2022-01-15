package model;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import dataStructures.DataHandler;
import dataStructures.OriginalGraph;
import dataStructures.SplitGraph;
import optimization.ExactMIP_VRPTR;
import optimization.SetPartitioning;
import tspHeuristics.RandomizedBestInsertionI;
import tspHeuristics.RandomizedFarthestInsertionI;
import tspHeuristics.RandomizedNearestInsertionI;
import tspHeuristics.RandomizedNFInsertion;
import tspHeuristics.RandomizedNearestNeighborI;
import tspHeuristics.RandomizedNearestNeighborNaive;
import utilities.EuclideanCalculator;

public class Solver {

	/**
	 * Replicate number
	 */
	
	private int replicate;
	
	/**
	 * Instance number
	 */
	private int instance;
	
	/**
	 * Best cost
	 */
	
	private double upperBound;
	
	/**
	 * Iteration limit for the sampling phase
	 */
	
	private int iterationsLimit;
	
	/**
	 * Time limit for the sampling phase
	 */
	
	private int timeLimit;
	
	/**
	 * Time limit for cplex
	 */
	
	private int timeLimitCplex;
	
	/**
	 * which tsp heuristics should we use
	 */
	
	private int whichTSPHeuristics;
	
	/**
	 * Computational time
	 */
	
	private static double computationalTime;
	
	/**
	 * This method creates a solver object
	 * @param inst
	 * @param itLimit
	 * @param timLimit
	 */
	public Solver() {
		instance = 0;
		timeLimit = 0;
		timeLimitCplex = 0;
		iterationsLimit = 0;
		upperBound = 999999;
	}
	
	/**
	 * This method runs the MA
	 * @param inst instance id
	 * @param itLimit iterations limit for the sampling phase
	 * @param timLimit time limit for the set partitioning
	 * @param cons print or not in console
	 * @param re replicate number
	 * @param fact randomization factor for the TSP heuristics
	 * @param tipc time limit for the set partitioning
	 * @param heur 0: if all TSP heuristics must be used. 1:rnn 2:rni 3:rfi 4:rbi
	 * @throws IOException
	 */
	public void MA(int in, int it, int ti,boolean cons,int re,int fact,int ticp,int heur)throws IOException {
		
		//-1. Starts the counter for the initialization step:
		
			Double IniTime = (double) System.nanoTime();
		
		//0. Updates the information selected by the user
		
			this.updateInfo(in, it, ti,re,ticp,heur);

		//1. Read the data:
			
			DataHandler data = this.readDataInfo(in);

		//2. Creates the graph:
				
			OriginalGraph graph = this.createGraph(data);

		//-2. Stops the counter:
				
			Double FinIniTime = (double) System.nanoTime();

		//3. Sampling procedure
			
			//3.3 Starts the clock:
			
				Double ITime = (double) System.nanoTime();	
				
			// Initializes an array:
				
				ArrayList<Integer> info = new ArrayList<Integer>();
			
			// Runs the sampling procedure:
				
				info = runSamplingStep_se1(data,graph,ITime,fact);
				
			Double FTime = (double) System.nanoTime();

		//4. Set partitioning model:
				
			//Starts the clock:
			
				Double ITimeS = (double) System.nanoTime();	
				
			// Runs the set partitioning model:
				
				int iteraciones = info.get(1);
				SetPartitioning optModel = new SetPartitioning(data,graph,info,info.get(0),re,heur,ticp);

			//Stops the clock:
				
				Double FTimeS = (double) System.nanoTime();		
			
		//5. Print a summary of results found:
			
			printSummary(IniTime,FinIniTime,ITime,FTime,ITimeS,FTimeS,iteraciones,optModel.getFinalCost(),optModel.getNumRoutesF(),in,data,re,heur,optModel.getSolutionIDs());
			if(cons) {
				printSummaryCons(IniTime,FinIniTime,ITime,FTime,ITimeS,FTimeS,iteraciones,optModel.getFinalCost(),optModel.getNumRoutesF(),in,data,re,heur,optModel.getSolutionIDs());
			}
			
	}
	
	/**
	 * This method runs the MIP for the VRPTR (with depot).
	 * @param in instance id
	 * @param cons should we print some info in console ?
	 * @param re replicate number of the initial solution 
	 * @param ticp time limit for the MIP
	 * @param heur set of heuristics used to compute the initial solution
	 * @param ini should we perform a warm start?
	 * @throws IOException
	 */
	public void MIP(int in,boolean cons,int ticp,boolean ini)throws IOException {
		
		//0. Updates the information selected by the user
		
			this.updateInfo(in, -1, -1,-1,ticp,-1);

		//1. Read the data:
			
			DataHandler data = this.readDataInfo(in);

		//2. Creates the graph:
				
			OriginalGraph graph = this.createGraph(data);

		//3. Runs the mip:
			
			ExactMIP_VRPTR mip = new ExactMIP_VRPTR(data,graph,timeLimitCplex,-1,-1,ini,cons);
			if(mip.getObjFunc() > 99999) {
				System.out.println("Wow");
			}
	}
	
	
	/**
	 * This method prints the final summary
	 * @param IniTime
	 * @param FinIniTime
	 * @param ITime
	 * @param FTime
	 * @param ITimeS
	 * @param FTimeS
	 * @param finalC
	 * @param finalN
	 * @param in
	 * @param data
	 */
	public void printSummary(Double IniTime,Double FinIniTime,Double ITime,Double FTime,Double ITimeS,Double FTimeS,int ite,double finalC,int finalN,int in,DataHandler data,int re,int heur,ArrayList<Integer>solutionIDs) {
		
		//Calculate how many routes belong to each type of TSP;
		
				int rnn = 0;
				int rni = 0;
				int rfi = 0;
				int rbi = 0;
				int rnf = 0;
				
		// Iterate over the routes id;
				
				for(Iterator<Integer> iter = solutionIDs.iterator();iter.hasNext();) {
					int act = iter.next();
					int typ = data.getRouteHeurs().get(act);
					if(typ == 1) {
						rnn++;
					}else if(typ == 2) {
						rni++;
					}else if(typ == 3) {
						rfi++;
					}else if(typ == 4) {
						rbi++;
					}else if(typ == 5) {
						rnn++;
					}else if(typ == 6) {
						rnf++;
					}
				}
				
		//Print the summary:
				
		String ruta = globalParameters.GlobalParameters.RESULT_FOLDER+"/Summary"+in+"-"+this.whichTSPHeuristics+"-"+re+".txt";
		try {
			PrintWriter pw = new PrintWriter(new File(ruta));
			pw.println("InitializationTime(s);"+((FinIniTime-IniTime)/1000000000));
			pw.println("SamplingTime(s);"+((FTime-ITime)/1000000000));
			pw.println("SetPartitioningTime(s);"+((FTimeS-ITimeS)/1000000000));
			pw.println("FinalCost;"+finalC);
			pw.println("FixedCost;"+finalN * data.getFixCost());
			pw.println("VariableCost;"+(finalC - finalN*data.getFixCost()));
			pw.println("VariableCost_in;"+(upperBound - finalN*data.getFixCost()));
			pw.println("Iterations;"+ite);
			pw.println("Routes;"+finalN);
			pw.println("RNN;"+rnn);
			pw.println("RNI;"+rni);
			pw.println("RFI;"+rfi);
			pw.println("RBI;"+rbi);
			pw.println("RNF;"+rnf);
			pw.close();
		}catch(Exception e) {
			System.out.println("Mistake printing the summary");
		}
	}
	
	/**
	 * This method prints the final summary
	 * @param IniTime
	 * @param FinIniTime
	 * @param ITime
	 * @param FTime
	 * @param ITimeS
	 * @param FTimeS
	 * @param finalC
	 * @param finalN
	 * @param in
	 * @param data
	 */
	public void printSummaryCons(Double IniTime,Double FinIniTime,Double ITime,Double FTime,Double ITimeS,Double FTimeS,int ite,double finalC,int finalN,int in,DataHandler data,int re,int heur,ArrayList<Integer>solutionIDs) {
		
		//Calculate how many routes belong to each type of TSP;
		
		int rnn = 0;
		int rni = 0;
		int rfi = 0;
		int rbi = 0;
		int rnf = 0;
		
		// Iterate over the routes id;
		
		for(Iterator<Integer> iter = solutionIDs.iterator();iter.hasNext();) {
			int act = iter.next();
			int typ = data.getRouteHeurs().get(act);
			//System.out.println(act+" - "+typ);
			if(typ == 1) {
				rnn++;
			}else if(typ == 2) {
				rni++;
			}else if(typ == 3) {
				rfi++;
			}else if(typ == 4) {
				rbi++;
			}else if(typ == 5) {
				rnn++;
			}else if(typ == 6) {
				rnf++;
			}
		}
		
		
		System.out.println("-----Instance "+instance+" ----- Replicate "+re);
		//System.out.println("InitializationTime(s);"+((FinIniTime-IniTime)/1000000000));
		System.out.println("SamplingTime(s);"+((FTime-ITime)/1000000000));
		System.out.println("SetPartitioningTime(s);"+((FTimeS-ITimeS)/1000000000));
		//System.out.println("FinalCost;"+finalC);
		//System.out.println("FixedCost;"+finalN * data.getFixCost());
		System.out.println("VariableCost;"+(finalC - finalN*data.getFixCost()));
		System.out.println("VariableCost_in;"+(upperBound - finalN*data.getFixCost()));
		System.out.println("Iterations;"+ite);
		System.out.println("Number of routes;"+finalN);
		if(rnn>0) {
			System.out.println(rnn+" routes come from a RNN");
		}
		if(rni>0) {
			System.out.println(rni+" routes come from a RNI");
		}
		if(rfi>0) {
			System.out.println(rfi+" routes come from a RFI");
		}
		if(rbi>0) {
			System.out.println(rbi+" routes come from a RBI");
		}
		if(rnf>0) {
			System.out.println(rnf+" routes come from a RFN");
		}
	}
	
	/**
	 * This method runs the sampling procedure for the set of instances 1
	 * @param data
	 * @param graph
	 * @param ITime
	 * @return
	 * @throws IOException
	 */
	public ArrayList<Integer> runSamplingStep_se1(DataHandler data, OriginalGraph graph,Double ITime,int fact) throws IOException {
		
		// Creates an array to store the ID's for the warm start
		
			ArrayList<Integer> sampR = new ArrayList<Integer>();
		
		//3.1 Creates a boolean variable to stop if necessary
			
			boolean end = true;
		
		//3.2 Number of iterations
			
			int ite = 0;
			int routes = 0 ;
			ArrayList<Integer> optRoutes = new ArrayList<Integer>();
		
		
		//3.4 Id's of the best solution found so far:
			
			ArrayList<Integer> bestIDs = new ArrayList<Integer>();
			int heur = -1; //Heuristic with the best solution so far.
			
		//Iterate until the timelimit or the iterations limit is met
				
			while(end && ite < iterationsLimit) {
					
			
			// Selects the tsp heuristic:
				
				int ran = this.getWhichTSPHeuristics();
				if(this.getWhichTSPHeuristics() == 0) {
					//Randomly selects one of the heuristics:
					
					ran = (int) Math.floor(Math.random()*6+1);
					
				}
				
			// Generates a tspTour:
						
				ArrayList<Integer> tspTour = generateTSP(graph,data,ran,fact);

			// Creates the split graph:
							
				SplitGraph sG = new SplitGraph(tspTour,graph,data,routes,ran);
				for(Iterator<Integer> iter = sG.getOptimalRoutes().iterator();iter.hasNext();) {
					optRoutes.add(iter.next());
				}
							
			// Checks if the solution is better than the last one:
							
				if(sG.recoverCost() < upperBound) {
					upperBound = sG.recoverCost();
					bestIDs = new ArrayList<Integer>();
					for(Iterator<Integer> iter = sG.getOptimalRoutes().iterator();iter.hasNext();) {
						bestIDs.add(iter.next());
					}
					heur = ran;
				}

				routes = sG.getNumRoutes();
							
				Double FTime = (double) System.nanoTime();
				if(((FTime-ITime)/1000000000) > timeLimit) {
				end = false;
				}
				ite++;
				resetEverything(graph);
				
			}
			
		//Stores the info
			
			sampR.add(routes);
			sampR.add(ite);
			sampR.add(heur);
			for(Iterator<Integer> iter = bestIDs.iterator();iter.hasNext();) {
				sampR.add(iter.next());
			}

		// Returns the array:
		
			return(sampR);
		
	}
	


	/**
	 * @return the instance
	 */
	public int getInstance() {
		return instance;
	}

	/**
	 * @param instance the instance to set
	 */
	public void setInstance(int instance) {
		this.instance = instance;
	}

	/**
	 * @return the upperBound
	 */
	public double getUpperBound() {
		return upperBound;
	}

	/**
	 * @param upperBound the upperBound to set
	 */
	public void setUpperBound(double upperBound) {
		this.upperBound = upperBound;
	}

	/**
	 * @return the iterationsLimit
	 */
	public int getIterationsLimit() {
		return iterationsLimit;
	}

	/**
	 * @param iterationsLimit the iterationsLimit to set
	 */
	public void setIterationsLimit(int iterationsLimit) {
		this.iterationsLimit = iterationsLimit;
	}

	
	public int getReplicate() {
		return replicate;
	}

	public void setReplicate(int replicate) {
		this.replicate = replicate;
	}

	/**
	 * @return the timeLimit
	 */
	public int getTimeLimit() {
		return timeLimit;
	}

	/**
	 * @param timeLimit the timeLimit to set
	 */
	public void setTimeLimit(int timeLimit) {
		this.timeLimit = timeLimit;
	}

	/**
	 * @return the computationalTime
	 */
	public static double getComputationalTime() {
		return computationalTime;
	}

	/**
	 * @param computationalTime the computationalTime to set
	 */
	public static void setComputationalTime(double computationalTime) {
		Solver.computationalTime = computationalTime;
	}
	
	    /**
		 * Resets the marks for the RNN
		 * @param G
		 */
		public static void resetEverything(dataStructures.OriginalGraph G) {
			G.resetMarks();
		}
		
		/**
		 * Updates instance information
		 * @param in
		 * @param it
		 * @param ti
		 */
		public void updateInfo(int in,int it, int ti,int re,int ticp,int heur) {
			this.setInstance(in);
			this.setIterationsLimit(it);
			this.setTimeLimit(ti);
			this.setReplicate(re);
			this.setTimeLimitCplex(ticp);
			this.setWhichTSPHeuristics(heur);
		}
		
		/**
		 * @return the timeLimitCplex
		 */
		public int getTimeLimitCplex() {
			return timeLimitCplex;
		}

		/**
		 * @param timeLimitCplex the timeLimitCplex to set
		 */
		public void setTimeLimitCplex(int timeLimitCplex) {
			this.timeLimitCplex = timeLimitCplex;
		}

		/**
		 * Creates the data handler
		 * @param in
		 * @return
		 * @throws IOException
		 */
		public DataHandler readDataInfo(int in) throws IOException {
			
			//1.0 Creates a data handler
			
			DataHandler data = new DataHandler(globalParameters.GlobalParameters.INSTANCE_FOLDER,in);
		
			//1.1 Read the Master file and stores the main parameters
		
			data.readMasterFile();
			
			//1.2 Read the graph and stores it.
		
			data.readCoordinates();
			
			//1.4 Returns the data handler
			
			return(data);
		}
		
		/**
		 * Creates the graph
		 * @param data
		 * @return
		 */
		public OriginalGraph createGraph(DataHandler data) {
			
		//2.1 Creates the original graph:
			
			
			OriginalGraph graph = new OriginalGraph(data.getxCoors().size(),data.getxCoors().size()*data.getxCoors().size());
			graph.createNodes(data.getxCoors().size());
		
			for(int i=0;i<data.getxCoors().size();i++) {
				data.getNodeRoutes().put((i),new ArrayList<Integer>());
			}
			
		//2.2 Calculates all arc's attributes:

				
			for(int tail=0;tail<data.getxCoors().size();tail++) {
				for(int head=0;head<data.getxCoors().size();head++) {
					double dis = EuclideanCalculator.calc(data.getxCoors().get(tail), data.getyCoors().get(tail), data.getxCoors().get(head), data.getyCoors().get(head));
					if(dis >  data.getMaxDistBTwoP()) {
						graph.addArc(tail, head, dis, 60*dis/(data.getDrivingSpeed()),  999999);
					}
					else {
						graph.addArc(tail, head, dis, 60*dis/(data.getDrivingSpeed()),  60*dis/(data.getWalkingSpeed()));
					}
				}
			}
	
		//2.3 Sorts all nodes
				
			graph.sortNodes();
			
			
		//2.4 Returns the graph
			
			return(graph);
		}
		
	/**
	 * This method generates a TSP with the CD
	 * @param graph
	 * @param data
	 * @ran tsp selected
	 * @param fact
	 * @return
	 */
		public ArrayList<Integer> generateTSP(OriginalGraph graph,DataHandler data,int ran,int fact){
			
			// Initializes the tspTour:
			
			ArrayList<Integer> tspTour = null;

			if(ran == 1) {
				RandomizedNearestNeighborI tspM = new RandomizedNearestNeighborI(graph,data,fact);
				tspTour = tspM.getTour();
			}
			else if(ran == 2) {
				RandomizedNearestInsertionI tspM = new RandomizedNearestInsertionI(graph,data,fact);
				tspTour = tspM.getTour();
			}
			else if(ran == 3) {
				RandomizedFarthestInsertionI tspM = new RandomizedFarthestInsertionI(graph,data,fact);
				tspTour = tspM.getTour();
			}
			else if(ran == 4) {
				RandomizedBestInsertionI tspM = new RandomizedBestInsertionI(graph,data,fact);
				tspTour = tspM.getTour();
			}else if(ran == 5) {
				RandomizedNearestNeighborNaive tspM = new RandomizedNearestNeighborNaive(graph,data,3);
				tspTour = tspM.getTour();
			}
			else {
				RandomizedNFInsertion tspM = new RandomizedNFInsertion(graph,data,fact);
				tspTour = tspM.getTour();
			}

			// Returns the tsp tour
			return(tspTour);
		}
		
		/**
		 * @return the whichTSPHeuristics
		 */
		public int getWhichTSPHeuristics() {
			return whichTSPHeuristics;
		}

		/**
		 * @param whichTSPHeuristics the whichTSPHeuristics to set
		 */
		public void setWhichTSPHeuristics(int whichTSPHeuristics) {
			this.whichTSPHeuristics = whichTSPHeuristics;
		}
		
		
}
