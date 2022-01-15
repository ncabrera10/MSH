package model;

import java.io.IOException;

/**
 * Class to manage the different algorithms
 */

public class Manager {

	public Manager() throws IOException, InterruptedException{

	}
	
	/**
	 * Runs the MA algorithm
	 */
	
	public Solver runMA(int inst, int itLimit, int timLimit,boolean cons,int re,int fa,int cplexTimeLimit,int heur)throws IOException, InterruptedException {
		
		// Creates a solver instance:
		Solver solver = new Solver();
		
		// Runs the MA:
		solver.MA(inst, itLimit, timLimit,cons,re,fa,cplexTimeLimit,heur);
		
		// Returns the solver instance:
		return solver;
	}
	
	/**
	 * Runs the MIP algorithm for the VRPTR
	 */
	
	public Solver runMIP(int inst,boolean cons,int cplexTimeLimit, boolean ini)throws IOException, InterruptedException {
		
		// Creates a solver instance:
		Solver solver = new Solver();
		
		// Runs the MA:
		solver.MIP(inst,cons,cplexTimeLimit,ini);
		
		// Returns the solver instance:
		return solver;
	}
	
}
