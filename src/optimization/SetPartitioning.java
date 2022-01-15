package optimization;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;

import dataStructures.DataHandler;
import dataStructures.OriginalGraph;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class SetPartitioning {

	/**
	 * ID's of the final selected routes
	 */
	private ArrayList<Integer> solutionIDs;
	
	/**
	 * Final cost found by the set partitioning model
	 */
	private double finalCost;
	
	/**
	 * Final number of routes:
	 */
	
	private int numRoutesF;
	
	/**
	 * Num nodes to consider for the constraints
	 */
	
	private int numNodesC;
	
	/**
	 * This method creates a class for performing the set partitioning model
	 * @param data
	 * @param graph
	 */
	public SetPartitioning(DataHandler data, OriginalGraph graph, ArrayList<Integer> info,int numRoutes,int re,int heur,int tice) {

		solutionIDs = new ArrayList<Integer>();
		info.remove(0);
		info.remove(0);
		info.remove(0);
		numNodesC = graph.getNumNodes()-1;
		
		try {
			runModel(data,graph,info,numRoutes,re,heur,tice);
		} catch (IloException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * This method runs the set partitioning model
	 * @throws IloException 
	 */
	public void runModel(DataHandler dataHandler,OriginalGraph G,ArrayList<Integer>info,int numRoutes,int re,int heur,int tice) throws IloException {
		
		//Creates a model
		
			IloCplex model = new IloCplex();
			
		//Creates the variables array
		
			IloNumVar[] vars = new IloNumVar[numRoutes];
		
		//Creates variables
		
			for(int i = 1;i<=numRoutes;i++) {
				vars[i-1] = model.boolVar();
			}
		
		//Creates constraints
		
			IloLinearNumExpr[] ctrs = new IloLinearNumExpr[numNodesC];
			for(int n = 0;n<numNodesC;n++){
				ctrs[n] = model.linearNumExpr();
				for(Iterator<Integer> iter = dataHandler.getNodeRoutes().get(n).iterator();iter.hasNext();) {
					int ruta = iter.next();
					ctrs[n].addTerm(1.0, vars[ruta-1]);
				}
			}
		
		//Creates the objective function
			
			IloLinearNumExpr objectivefunction = model.linearNumExpr();
			
			for(int i = 1;i<=numRoutes;i++) {
				objectivefunction.addTerm(dataHandler.getRouteCosts().get(i), vars[i-1]);
			}
			
			model.addMinimize(objectivefunction);
			
		//Add all constraints
			
			for(int n = 0;n<numNodesC;n++){
				model.addEq(ctrs[n], 1);
			}
		
		
		//Try warm start
			
			double[] iniValues = new double[numRoutes];
			for(int i = 1;i<=numRoutes;i++) {
				iniValues[i-1] = 0;
			}
			for(Iterator<Integer> iter = info.iterator();iter.hasNext();) {
				iniValues[iter.next()-1] = 1;
			}
			try {
				model.addMIPStart(vars, iniValues);
			}
			catch(Exception e){
				System.out.println("Error con la inicializacion");
				System.out.println(e.getStackTrace());
				System.out.println(e.getMessage());
				e.printStackTrace();
			}
			
		//Optimize
			
		   model.setOut(null);
		   model.setParam(IloCplex.Param.TimeLimit,tice);
		
			if(model.solve()) {

				
				this.setFinalCost(model.getObjValue());
				
				String ruta = globalParameters.GlobalParameters.RESULT_FOLDER+"RouteArcs"+dataHandler.getInst()+"-"+heur+"-"+re+".txt";
				String ruta2 = globalParameters.GlobalParameters.RESULT_FOLDER+"RouteStrings"+dataHandler.getInst()+"-"+heur+"-"+re+".txt";
				String rutaA = globalParameters.GlobalParameters.RESULT_FOLDER+"Auxiliar"+"/RouteArcs"+dataHandler.getInst()+"-"+(-1)+"-"+(-1)+".txt";
				String ruta2A = globalParameters.GlobalParameters.RESULT_FOLDER+"Auxiliar"+"/RouteStrings"+dataHandler.getInst()+"-"+(-1)+"-"+(-1)+".txt";
				
				try {
					PrintWriter pw = new PrintWriter(new File(ruta));
					PrintWriter pw2 = new PrintWriter(new File(ruta2));
					PrintWriter pwA = new PrintWriter(new File(rutaA));
					PrintWriter pw2A = new PrintWriter(new File(ruta2A));
					for(int i = 1;i<=numRoutes;i++) {
						//System.out.println(model.getValue(vars[i-1]));
						if(model.getValue(vars[i-1]) > 0.5) {
							solutionIDs.add(i);
							numRoutesF++;
							pw2.println(i+";"+dataHandler.getRouteHeurs().get(i)+";"+dataHandler.getRouteStrings().get(i)+";"+dataHandler.getRouteCosts().get(i)+";"+dataHandler.getRouteTimes().get(i));
							pw2A.println(i+";"+dataHandler.getRouteHeurs().get(i)+";"+dataHandler.getRouteStrings().get(i)+";"+dataHandler.getRouteCosts().get(i)+";"+dataHandler.getRouteTimes().get(i));
							
								for(Iterator<String> iter = dataHandler.getRoutePrints().get(i).iterator();iter.hasNext();) {
									String act = iter.next();
									String[] vals = act.split(";");
									if(vals[1].equals("-1")) {
										pw.println((Integer.parseInt(vals[0])+1)+";"+G.getNumNodes()+";"+vals[2]+";"+i);
										pwA.println((Integer.parseInt(vals[0])+1)+";"+G.getNumNodes()+";"+vals[2]+";"+i);
										
									}else if(!vals[0].equals("-1")) {
										pw.println((Integer.parseInt(vals[0])+1)+";"+(Integer.parseInt(vals[1])+1)+";"+vals[2]+";"+i);
										pwA.println((Integer.parseInt(vals[0])+1)+";"+(Integer.parseInt(vals[1])+1)+";"+vals[2]+";"+i);
			
									}
								}
						}
					}
					pw.close();
					pw2.close();
					pwA.close();
					pw2A.close();
					model.close();
				}
				catch(Exception e) {
					System.out.println(e.getStackTrace());
					System.out.println(e.getMessage());
					e.printStackTrace();
					System.out.println("Error printing results of the set partitioning ");
				}
				
		}
	}

	/**
	 * @return the solutionIDs
	 */
	public ArrayList<Integer> getSolutionIDs() {
		return solutionIDs;
	}

	/**
	 * @param solutionIDs the solutionIDs to set
	 */
	public void setSolutionIDs(ArrayList<Integer> solutionIDs) {
		this.solutionIDs = solutionIDs;
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
	 * @return the numRoutesF
	 */
	public int getNumRoutesF() {
		return numRoutesF;
	}

	/**
	 * @param numRoutesF the numRoutesF to set
	 */
	public void setNumRoutesF(int numRoutesF) {
		this.numRoutesF = numRoutesF;
	}

	
	
}
