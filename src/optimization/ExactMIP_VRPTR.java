package optimization;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import java.util.Hashtable;


import dataStructures.DataHandler;
import dataStructures.OriginalGraph;
import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

/**
 * This class solves a MIP to solve the VRPTR
 * @author nick0
 *
 */
public class ExactMIP_VRPTR {

	/**
	 * 1: if the worker drives from i to j
	 */
	private Hashtable<String,IloNumVar> x;
	
	/**
	 * 1: if the worker walks from i to j
	 */
	private Hashtable<String,IloNumVar> h;
	
	/**
	 * 1: if the worker starts at i
	 */
	private Hashtable<String,IloNumVar> y;
	
	/**
	 * 1: if the worker ends at i
	 */
	private Hashtable<String,IloNumVar> z;
	
	/**
	 * 1: if the worker parks the car at i
	 */
	private Hashtable<String,IloNumVar> g;
	
	/**
	 * Arrival time at i for worker e
	 */
	private Hashtable<String,IloNumVar> w;
	
	/**
	 * 1: if worker e attends customer i
	 */
	private Hashtable<String,IloNumVar> b;
	
	/**
	 * 1: if worker e is selected
	 */
	private Hashtable<String,IloNumVar> a;
	
	/**
	 * Total number of nodes (with the phantoms)
	 */
	private int N;
	
	/**
	 * Number of nodes + 2 (the depot)
	 */
	private int n;
	
	/**
	 * Pos in x (km)
	 */
	private Hashtable<Integer,Double> posx;
	
	/**
	 * Pos in y (km)
	 */
	private Hashtable<Integer,Double> posy;
	
	/**
	 * Service time in min
	 */
	private Hashtable<Integer,Integer> serv;
	
	/**
	 * If the arc exists or not
	 */
	private Hashtable<String,Integer> l; //1: If the arc between i and j exists 0: otherwise
	
	/**
	 * Walking times
	 */
	private Hashtable<String,Double> wt; //1:Walking time of arc between i and j
	
	/**
	 * Walking distances
	 */
	private Hashtable<String,Double> wd; //1:Walking distance of arc between i and j
	
	/**
	 * Driving times
	 */
	private Hashtable<String,Double> dt; //1:Driving time of arc between i and j
	
	/**
	 * Driving distances
	 */
	private Hashtable<String,Double> dd; //1:Driving time of arc between i and j
	

	/**
	 * Number of workers available
	 * 
	 */
	private int m;
	
	/**
	 * Variable cost of hiring
	 */
	private double cv;
	
	/**
	 * Fixed cost of hiring
	 */
	private double cf;
	
	/**
	 * Maximum duration of a subtour
	 */
	private double TS;
	
	/**
	 * Maximum distance between two points
	 */
	private int md;
	
	/**
	 * Big M
	 */
	private int M;
	
	/**
	 * Workers day duration
	 */
	private int T;
	
	/**
	 * Maximum walking distance in a day
	 */
	private int D;
	
	/**
	 * Driving speed
	 */
	private double ds;
	
	/**
	 * Walking speed
	 */
	private double ws;
	
	/**
	 * Total service time
	 */
	private double totalServ;
	
	/**
	 * Objective function
	 */
	private double objFunc = 99999;
	
	/**
	 * The model of cplex
	 */
	private IloCplex model;
	
	/**
	 * This class creates a ExactMIP object
	 * @param data
	 * @param graph
	 */
	public ExactMIP_VRPTR(DataHandler data, OriginalGraph graph,int tice,int re,int heur,boolean ini,boolean cons) {
		
		//Create parameters:
		
		createParameters(data,graph);
		
		//Runs the model:
		
		try {
			runModel(data,graph, tice,re,heur,ini,cons);
		} catch (IloException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	
	public void runModel(DataHandler data,OriginalGraph graph,int tice,int re,int heur,boolean ini,boolean cons) throws IloException {
		
		//1. Creates a model
		
		model = new IloCplex();
		
		//2. Creates variables:
		
		createVariables(data,graph);
		
		//3. Creates constraints:
		
		createConstraints(data,graph);
		
		//4. Creates the objective function:
		
		IloLinearNumExpr objectivefunction = createObjectiveFunction();

		//5. Set the objective:
		
		model.addMinimize(objectivefunction);

		//6. Main parameters:
		
		double[] iniValues = null;
		IloNumVar[] vars = null;

		try{
			
			if(ini) {
				//System.out.println("Leyendo sol inicial");
				Hashtable<Integer,Object> solIni = readInitialSolutionTxt(data,re,heur);
				iniValues = (double[]) solIni.get(1);
				vars = (IloNumVar[]) solIni.get(2);
				//String[] cuales = (String[]) solIni.get(3);
				/**
				for(int i = 0;i<iniValues.length;i++) {
					if(iniValues[i]>0) {
						System.out.println(cuales[i]+" - "+iniValues[i]);
					}
					
				}
				*/
				model.addMIPStart(vars,iniValues);
				model.setParam(IloCplex.Param.Emphasis.MIP, 3);

			}
			
		}
		catch(Exception e) {
			e.printStackTrace();
			System.out.println("Problem while reading initial sol********");
		}
		
		model.setParam(IloCplex.Param.TimeLimit,tice);
		if(!cons) {
			model.setOut(null);	
		}
		

		//7. Optimize:
		
		if(model.solve()) {
			
			// Store the objective function:
			
			objFunc = model.getObjValue();

			//Print results:
			
			printResults(data,re);
				

		}
		
		//8. Close the model:
		
		model.close();
	}


	/**
	 * @return the objFunc
	 */
	public double getObjFunc() {
		return objFunc;
	}


	/**
	 * @param objFunc the objFunc to set
	 */
	public void setObjFunc(double objFunc) {
		this.objFunc = objFunc;
	}
	
	public void printResults(DataHandler data,int re)throws IloException{
		
		
		
		//Print the solution for the app:
		
		String ruta = "./results/"+"mip"+"/RouteArcs"+data.getInst()+".txt";
		String ruta2 = "./results/"+"mip"+"/Details"+data.getInst()+".txt";
		String ruta3 = "./results/"+"mip"+"/Summary"+data.getInst()+".txt";
		
		try {
			PrintWriter pw = new PrintWriter(new File(ruta));
			PrintWriter pw2 = new PrintWriter(new File(ruta2));
			PrintWriter pw3 = new PrintWriter(new File(ruta3));
			int cant = 0;
			for(int e=1;e<=m;e++) {
				if(model.getValue(a.get(""+e)) > 0.5) {
					
					cant++;
				}
			}
			pw3.println("InitializationTime(s);"+(-1));
			pw3.println("SamplingTime(s);"+(-1));
			pw3.println("SetPartitioningTime(s);"+(-1));
			pw3.println("FinalCost;"+(model.getObjValue()));
			pw3.println("FixedCost;"+(cf * cant));
			pw3.println("VariableCost;"+(model.getObjValue() - (cant*cf)));
			pw3.println("BestLowerBound;"+(model.getBestObjValue()));
			pw3.println("Iterations;"+(-1));
			pw3.println("Routes;"+cant);
			pw3.println("RNN;"+(-1));
			pw3.println("RNI;"+(-1));
			pw3.println("RFI;"+(-1));
			pw3.println("RBI;"+(-1));
			pw3.println("RNF;"+(-1));
			
			pw2.println("With depot");
			pw2.println("*******************************");
			pw2.println("The total cost is "+(model.getObjValue()));
			pw2.println("The fixed cost is: "+(cant*cf));
			pw2.println("The variable cost is: "+(model.getObjValue() - (cant*cf)));
			pw2.println();
			pw2.println("Routes info: ");
			for(int e=1;e<=m;e++) {
				if(model.getValue(a.get(""+e)) > 0.5) {
					pw2.println("Worker "+e+":");
					for(int i=1;i<=N;i++) {
						if(y.containsKey(i+"-"+e)) {
							if(model.getValue(y.get(i+"-"+e))>0.5) {
								pw2.println("\t Departure hour from node "+i+":"+model.getValue(w.get(i+"-"+e)));
							}
						}
					}
					for(int i=1;i<=N;i++) {
						if(z.containsKey(i+"-"+e)) {
							if(model.getValue(z.get(i+"-"+e))>0.5) {
								pw2.println("\t Arrival hour to node "+i+":"+model.getValue(w.get(i+"-"+e)));
							}
						}
					}
					for(int i=1;i<=N;i++) {
						if(g.containsKey(i+"-"+e)) {
							if(model.getValue(g.get(i+"-"+e))>0.5) {
								pw2.println("\t Node used as parking spot"+i+":"+model.getValue(g.get(i+"-"+e)));
							}
						}
					}
					
					double totwd = 0;
					double totdd = 0;
					double totserv = 0;
					for(int i=1;i<=N;i++) {
						for(int j=1;j<=N;j++) {
							
							if(h.containsKey(i+"-"+j+"-"+e)) {

								totwd = totwd + model.getValue(h.get(i+"-"+j+"-"+e)) * wd.get(i+"-"+j);
							}
							if(x.containsKey(i+"-"+j+"-"+e)) {
								totdd = totdd + model.getValue(x.get(i+"-"+j+"-"+e)) * dd.get(i+"-"+j);
							}
						}
					}
					for(int i=1;i<=n;i++) {
						if(b.containsKey(i+"-"+e)) {
							totserv = totserv + model.getValue(b.get(i+"-"+e))*serv.get(i);
						}
					}
					pw2.println("\t Total service time: "+totserv);
					pw2.println("\t Total walking distance: "+totwd);
					pw2.println("\t Total driving distance: "+totdd);
					
					pw2.println("");
					for(int i=1;i<=N;i++) {
						for(int j=1;j<=N;j++) {
							if(h.containsKey(i+"-"+j+"-"+e)) {
								if(model.getValue(h.get(i+"-"+j+"-"+e)) > 0.5) {
									if(i<=n && j<=n) {
										pw2.println("\t"+i+" --- "+j+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" h_"+i+"-"+j+"-"+e);
									}	
									if(i<=n && j>n) {
										pw2.println("\t"+i+" --- "+(j-n)+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" h_"+i+"-"+j+"-"+e);
									}	
									if(i>n && j<=n) {
										pw2.println("\t"+(i-n)+" --- "+j+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" h_"+i+"-"+j+"-"+e);
									}	
								}
							}
							if(x.containsKey(i+"-"+j+"-"+e)) {
								if(model.getValue(x.get(i+"-"+j+"-"+e)) > 0.5) {
									if(i<=n && j<=n) {
										pw2.println("\t"+i+" -> "+j+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" x_"+i+"-"+j+"-"+e);
									}	
									if(i<=n && j>n) {
										pw2.println("\t"+i+" -> "+(j-n)+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" x_"+i+"-"+j+"-"+e);
									}	
									if(i>n && j<=n) {
										pw2.println("\t"+(i-n)+" -> "+j+" - ("+model.getValue(w.get(i+"-"+e))+") - "+dt.get(i+"-"+j)+" x_"+i+"-"+j+"-"+e);
									}	
								}
							}
						}
					}
				}
			}

			for(int e=1;e<=m;e++) {
				if(model.getValue(a.get(""+e)) > 0.5) {
					for(int i=1;i<=N;i++) {
						for(int j=1;j<=N;j++) {
							if(x.containsKey(i+"-"+j+"-"+e)) {
								if(model.getValue(x.get(i+"-"+j+"-"+e)) > 0.5) {
									if(i<=n && j<=n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-1+1)+";"+1+";"+e);
										}
										if(j == n) {
											pw.println((i-1+1)+";"+(n-1)+";"+1+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-1+1)+";"+(j-1+1)+";"+1+";"+e);
										}
									}
									if(i<=n && j>n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-n-1+1)+";"+1+";"+e);
										}
										if(j == n) {
											pw.println((i-1+1)+";"+(n-1)+";"+1+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-1+1)+";"+(j-n-1+1)+";"+1+";"+e);
										}
									}
									if(i>n && j<=n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-1+1)+";"+1+";"+e);
										}
										if(j == n) {
											pw.println((i-n-1+1)+";"+(n-1)+";"+1+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-n-1+1)+";"+(j-1+1)+";"+1+";"+e);
										}
									}
								}
							}
							if(h.containsKey(i+"-"+j+"-"+e)) {
								if(model.getValue(h.get(i+"-"+j+"-"+e)) > 0.5) {
									if(i<=n && j<=n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-1+1)+";"+2+";"+e);
										}
										if(j == n) {
											pw.println((i-1+1)+";"+(n-1)+";"+2+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-1+1)+";"+(j-1+1)+";"+2+";"+e);
										}
									}
									if(i<=n && j>n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-n-1+1)+";"+2+";"+e);
										}
										if(j == n) {
											pw.println((i-1+1)+";"+(n-1)+";"+2+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-1+1)+";"+(j-n-1+1)+";"+2+";"+e);
										}
									}
									if(i>n && j<=n) {
										if(i == n) {
											pw.println((n-1)+";"+(j-1+1)+";"+2+";"+e);
										}
										if(j == n) {
											pw.println((i-n-1+1)+";"+(n-1)+";"+2+";"+e);
										}
										if(i!=n && j!=n) {
											pw.println((i-n-1+1)+";"+(j-1+1)+";"+2+";"+e);
										}
									}
								}
							}
						}
					}
				}
			}
			pw.close();
			pw2.close();
			pw3.close();
		}
		catch(Exception e) {
			System.out.println("Error while printing");
		}
		
	}
	
	/**
	 * Creates the objective function
	 * @return
	 * @throws IloException
	 */
	public IloLinearNumExpr createObjectiveFunction()throws IloException{
		
		//4. Objective function:
		
				IloLinearNumExpr objectivefunction = model.linearNumExpr();
					
				for(int e=1;e<=m;e++) {
					
					for(int i=1;i<=N;i++) {
						
						for(int j=1;j<=N;j++) {
							
							if(l.containsKey(i+"-"+j) && x.containsKey(i+"-"+j+"-"+e)) {

								objectivefunction.addTerm(dd.get(i+"-"+j)*cv,x.get(i+"-"+j+"-"+e));
								
							}
						}
					}
					objectivefunction.addTerm(cf,a.get(""+e));
				}
				
				return(objectivefunction);
	}
	
	/**
	 * 
	 * @param data
	 * @param graph
	 * @throws IloException
	 */
	public void createConstraints(DataHandler data,OriginalGraph graph)throws IloException{
		
		//6.1 Employees flowing:
		
		Hashtable<String,IloLinearNumExpr> ctrs1 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				ctrs1.put(e+"-"+i,model.linearNumExpr());
				if(y.containsKey(i+"-"+e)) {
					ctrs1.get(e+"-"+i).addTerm(1,y.get(i+"-"+e));
				}
				if(z.containsKey(i+"-"+e)) {
					ctrs1.get(e+"-"+i).addTerm(-1,z.get(i+"-"+e));
				}
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(x.containsKey(i+"-"+j+"-"+e)) {
							ctrs1.get(e+"-"+i).addTerm(-1,x.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs1.get(e+"-"+i).addTerm(-1,h.get(i+"-"+j+"-"+e));
						}
					}
				}
				for(int j=1;j<=N;j++) {
					if(l.containsKey(j+"-"+i)) {
						if(x.containsKey(j+"-"+i+"-"+e)) {
							ctrs1.get(e+"-"+i).addTerm(1,x.get(j+"-"+i+"-"+e));
						}
						if(h.containsKey(j+"-"+i+"-"+e)) {
							ctrs1.get(e+"-"+i).addTerm(1,h.get(j+"-"+i+"-"+e));
						}
					}
				}
				model.addEq(ctrs1.get(e+"-"+i),0);
			}
			
		}
		
	//6.2 Employees must start and end (If they are selected)
		
		Hashtable<String,IloLinearNumExpr> ctrs2 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs3 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			ctrs2.put(""+e,model.linearNumExpr());
			ctrs3.put(""+e,model.linearNumExpr());
			for(int i=1;i<=N;i++) {
				if(y.containsKey(i+"-"+e)) {
					ctrs2.get(""+e).addTerm(1,y.get(i+"-"+e));
					ctrs3.get(""+e).addTerm(1,y.get(i+"-"+e));
				}
				if(z.containsKey(i+"-"+e)) {
					ctrs2.get(""+e).addTerm(-1,z.get(i+"-"+e));
				}	
			}
			ctrs3.get(""+e).addTerm(-1,a.get(""+e));
			model.addEq(ctrs2.get(""+e),0);
			model.addEq(ctrs3.get(""+e),0);

		}
		
	//6.3 Relation between a,x and h:
		
		Hashtable<String,IloLinearNumExpr> ctrs4 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			ctrs4.put(""+e,model.linearNumExpr());
			ctrs4.get(""+e).addTerm(l.values().size(), a.get(""+e));
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(x.containsKey(i+"-"+j+"-"+e)) {
							ctrs4.get(""+e).addTerm(-1, x.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs4.get(""+e).addTerm(-1, h.get(i+"-"+j+"-"+e));
						}
					}
				}
			}
			model.addGe(ctrs4.get(""+e), 0);
		}
		
		Hashtable<String,IloLinearNumExpr> ctrs4_b = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			ctrs4_b.put(""+e,model.linearNumExpr());
			ctrs4_b.get(""+e).addTerm(T, a.get(""+e));
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(x.containsKey(i+"-"+j+"-"+e)) {
							ctrs4_b.get(""+e).addTerm(-dt.get(i+"-"+j), x.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs4_b.get(""+e).addTerm(-wt.get(i+"-"+j), h.get(i+"-"+j+"-"+e));
						}
					}
				}
			}
			for(int i=1;i<=N;i++) {
				if(b.containsKey(i+"-"+e)) {
					ctrs4_b.get(""+e).addTerm(-serv.get(i), b.get(i+"-"+e));
				}
			}
			model.addGe(ctrs4_b.get(""+e), 0);
		}

		Hashtable<String,IloLinearNumExpr> ctrs5_b = new Hashtable<String,IloLinearNumExpr>();
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						ctrs5_b.put(i+"-"+j,model.linearNumExpr());
						for(int e=1;e<=m;e++) {
							if(x.containsKey(i+"-"+j+"-"+e)) {
								ctrs5_b.get(i+"-"+j).addTerm(1, x.get(i+"-"+j+"-"+e));
							}
							if(x.containsKey(j+"-"+i+"-"+e)) {
								ctrs5_b.get(i+"-"+j).addTerm(1, x.get(j+"-"+i+"-"+e));
							}
						}
						model.addLe(ctrs5_b.get(i+"-"+j), 1);
					}
				}
			}
			
		
		Hashtable<String,IloLinearNumExpr> ctrs6_b = new Hashtable<String,IloLinearNumExpr>();
		for(int i=1;i<=N;i++) {
			for(int j=1;j<=N;j++) {
				if(l.containsKey(i+"-"+j)) {
					ctrs6_b.put(i+"-"+j,model.linearNumExpr());
					for(int e=1;e<=m;e++) {
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs6_b.get(i+"-"+j).addTerm(1, h.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(j+"-"+i+"-"+e)) {
							ctrs6_b.get(i+"-"+j).addTerm(1, h.get(j+"-"+i+"-"+e));
						}
					}
					model.addLe(ctrs6_b.get(i+"-"+j), 1);
				}
			}
		}
		
	//6.4 Arrival time to node i.
		
		Hashtable<String,IloLinearNumExpr> ctrs5 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						ctrs5.put(i+"-"+j+"-"+e,model.linearNumExpr());
						if(w.containsKey(i+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(1, w.get(i+"-"+e));
						}
						if(b.containsKey(i+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(serv.get(i), b.get(i+"-"+e));
						}
						if(x.containsKey(i+"-"+j+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(dt.get(i+"-"+j), x.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(wt.get(i+"-"+j), h.get(i+"-"+j+"-"+e));
						}
						if(w.containsKey(j+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(-1, w.get(j+"-"+e));
						}
						if(x.containsKey(i+"-"+j+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(M, x.get(i+"-"+j+"-"+e));
						}
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs5.get(i+"-"+j+"-"+e).addTerm(M, h.get(i+"-"+j+"-"+e));
						}
						model.addLe(ctrs5.get(i+"-"+j+"-"+e), M);
						
					}
				}
			}
			
		}

	//6.5 Departure hour for techinician e
		
		Hashtable<String,IloLinearNumExpr> ctrs6 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				ctrs6.put(i+"-"+e,model.linearNumExpr());
				if(y.containsKey(i+"-"+e)) {
					ctrs6.get(i+"-"+e).addTerm(T, y.get(i+"-"+e));
				}
				if(w.containsKey(i+"-"+e)) {
					ctrs6.get(i+"-"+e).addTerm(1, w.get(i+"-"+e));
				}
				model.addLe(ctrs6.get(i+"-"+e),T);
			}
		}
		
	//6.6 If you park your car, you must walk !

		Hashtable<String,IloLinearNumExpr> ctrs7 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs8 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs9 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs10 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=n;i++) {
				ctrs7.put(i+"-"+e,model.linearNumExpr());
				ctrs8.put(i+"-"+e,model.linearNumExpr());
				ctrs9.put(i+"-"+e,model.linearNumExpr());
				for(int j=1;j<=n;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(h.containsKey(i+"-"+j+"-"+e)) {
							ctrs7.get(i+"-"+e).addTerm(1, h.get(i+"-"+j+"-"+e));
							ctrs8.get(i+"-"+e).addTerm(1, h.get(i+"-"+j+"-"+e));
						}
					}
					if(l.containsKey(j+"-"+(i+n))) {
						if(h.containsKey((j)+"-"+(i+n)+"-"+e)) {
							ctrs9.get(i+"-"+e).addTerm(1, h.get((j)+"-"+(i+n)+"-"+e));
						}
						
					}
				}
				
				if(g.containsKey(i+"-"+e)) {
					ctrs8.get(i+"-"+e).addTerm(-2, g.get(i+"-"+e));
					ctrs9.get(i+"-"+e).addTerm(-1, g.get(i+"-"+e));
				}

				//model.addLe(ctrs7.get(i+"-"+e),0);
				//model.addGe(ctrs8.get(i+"-"+e),-1);
				model.addEq(ctrs9.get(i+"-"+e),0);

			}
		}
		
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				ctrs10.put(i+"-"+e,model.linearNumExpr());
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+(j))) {
						if(h.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs10.get(i+"-"+e).addTerm(1, h.get((i)+"-"+(j)+"-"+e));
						}
						if(x.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs10.get(i+"-"+e).addTerm(1, x.get((i)+"-"+(j)+"-"+e));
						}
						
					}
					
				}
				model.addLe(ctrs10.get(i+"-"+e),1);
			}
		}

	//6.7 You either walk or drive, you dont do both

		Hashtable<String,IloLinearNumExpr> ctrs11 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						ctrs11.put(i+"-"+j+"-"+e,model.linearNumExpr());
						if(h.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs11.get(i+"-"+j+"-"+e).addTerm(1, h.get((i)+"-"+(j)+"-"+e));
						}
						if(x.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs11.get(i+"-"+j+"-"+e).addTerm(1, x.get((i)+"-"+(j)+"-"+e));
						}
						model.addLe(ctrs11.get(i+"-"+j+"-"+e),1);
					}
				}
			}
		}
		
	//6.8 Subtour duration limit

		Hashtable<String,IloLinearNumExpr> ctrs12 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=n;i++) {
				ctrs12.put(i+"-"+e,model.linearNumExpr());
				if(w.containsKey((i+n)+"-"+e)) {
					ctrs12.get(i+"-"+e).addTerm(1, w.get((i+n)+"-"+e));
				}
				if(w.containsKey((i)+"-"+e)) {
					ctrs12.get(i+"-"+e).addTerm(-1, w.get((i)+"-"+e));
				}
				if(b.containsKey((i)+"-"+e)) {
					ctrs12.get(i+"-"+e).addTerm(-serv.get(i), b.get((i)+"-"+e));
				}
				if(g.containsKey((i)+"-"+e)) {
					ctrs12.get(i+"-"+e).addTerm((-1*TS+M), g.get((i)+"-"+e));
				}
				
				model.addLe(ctrs12.get(i+"-"+e),M);
			}
		}

	//6.9 !10. Walking distance limit

		Hashtable<String,IloLinearNumExpr> ctrs13 = new Hashtable<String,IloLinearNumExpr>();
		for(int e=1;e<=m;e++) {
			ctrs13.put(""+e,model.linearNumExpr());
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(h.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs13.get(""+e).addTerm(dd.get(i+"-"+j), h.get((i)+"-"+(j)+"-"+e));
						}
					}
				}
			}
			ctrs13.get(""+e).addTerm(-D,a.get(""+e));
			
			model.addLe(ctrs13.get(""+e),0);
		}

	//!11. All clients must be attended

		Hashtable<String,IloLinearNumExpr> ctrs14 = new Hashtable<String,IloLinearNumExpr>();
		for(int i=1;i<=n-2;i++) {
			ctrs14.put(""+i,model.linearNumExpr());
			for(int e=1;e<=m;e++) {
				if(b.containsKey(i+"-"+e)){
					ctrs14.get(""+i).addTerm(1, b.get(i+"-"+e));
				}
			}
			model.addEq(ctrs14.get(""+i),1);
		}
		
		Hashtable<String,IloLinearNumExpr> ctrs15 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs16 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int i=1;i<=n;i++) {
			for(int e=1;e<=m;e++) {
				ctrs15.put(i+"-"+e,model.linearNumExpr());
				ctrs16.put(i+"-"+e,model.linearNumExpr());
				if(b.containsKey(i+"-"+e)){
					ctrs15.get(i+"-"+e).addTerm(1, b.get(i+"-"+e));
					ctrs16.get(i+"-"+e).addTerm(1, b.get(i+"-"+e));
				}
				if(y.containsKey(i+"-"+e)){
					ctrs15.get(i+"-"+e).addTerm(-1, y.get(i+"-"+e));
				}
				for(int j=1;j<=N;j++) {
					if(l.containsKey(j+"-"+i)) {
						if(h.containsKey((j)+"-"+(i)+"-"+e)) {
							ctrs15.get(i+"-"+e).addTerm(-1, h.get((j)+"-"+(i)+"-"+e));
							ctrs16.get(i+"-"+e).addTerm(-1, h.get((j)+"-"+(i)+"-"+e));
						}
						if(x.containsKey((j)+"-"+(i)+"-"+e)) {
							ctrs15.get(i+"-"+e).addTerm(-1, x.get((j)+"-"+(i)+"-"+e));
							ctrs16.get(i+"-"+e).addTerm(-1, x.get((j)+"-"+(i)+"-"+e));
						}
					}
				}
				model.addLe(ctrs15.get(i+"-"+e),0);
				model.addGe(ctrs16.get(i+"-"+e),0);
			}
		}

					
						
	//12. When you walk somewhere you continue walking..until you go back to the car

		Hashtable<String,IloLinearNumExpr> ctrs17 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs18 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int i=1;i<=n;i++) {
			for(int e=1;e<=m;e++) {
				ctrs17.put(i+"-"+e,model.linearNumExpr());
				ctrs18.put(i+"-"+e,model.linearNumExpr());
				if(g.containsKey(i+"-"+e)){
					ctrs17.get(i+"-"+e).addTerm(-1, g.get(i+"-"+e));
					ctrs18.get(i+"-"+e).addTerm(1, g.get(i+"-"+e));
				}
				/**
				if(y.containsKey(i+"-"+e)){
					ctrs17.get(i+"-"+e).addTerm(-1*M, y.get(i+"-"+e));
					ctrs18.get(i+"-"+e).addTerm(M, y.get(i+"-"+e));
				}
				*/
				for(int j=1;j<=N;j++) {
					if(l.containsKey(j+"-"+i)) {
						if(h.containsKey((j)+"-"+(i)+"-"+e)) {
							ctrs17.get(i+"-"+e).addTerm(-1, h.get((j)+"-"+(i)+"-"+e));
							ctrs18.get(i+"-"+e).addTerm(-1, h.get((j)+"-"+(i)+"-"+e));
						}
					}
				}
				for(int j=1;j<=N;j++) {
					if(l.containsKey(i+"-"+j)) {
						if(h.containsKey((i)+"-"+(j)+"-"+e)) {
							ctrs17.get(i+"-"+e).addTerm(1, h.get((i)+"-"+(j)+"-"+e));
							ctrs18.get(i+"-"+e).addTerm(1, h.get((i)+"-"+(j)+"-"+e));
						}
					}
				}
				model.addLe(ctrs17.get(i+"-"+e),0);
				model.addGe(ctrs18.get(i+"-"+e),0);
			}
		}

	//13. By now..you cant start with a subtour

		Hashtable<String,IloLinearNumExpr> ctrs19 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int i=1;i<=N;i++) {
			for(int e=1;e<=m;e++) {
				ctrs19.put(i+"-"+e,model.linearNumExpr());
				if(y.containsKey(i+"-"+e)){
					ctrs19.get(i+"-"+e).addTerm(1, y.get(i+"-"+e));
				}
				if(g.containsKey(i+"-"+e)){
					ctrs19.get(i+"-"+e).addTerm(1, g.get(i+"-"+e));
				}
				model.addLe(ctrs19.get(i+"-"+e),2);
			}
		}

	//14. Constraints for the depot !!!

		Hashtable<String,IloLinearNumExpr> ctrs20 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs21 = new Hashtable<String,IloLinearNumExpr>();
		Hashtable<String,IloLinearNumExpr> ctrs22 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int e=1;e<=m;e++) {
			ctrs20.put(""+e,model.linearNumExpr());
			ctrs21.put(""+e,model.linearNumExpr());
			ctrs22.put(""+e,model.linearNumExpr());
			
			ctrs20.get(""+e).addTerm(1,y.get((n-1)+"-"+e));
			ctrs21.get(""+e).addTerm(1,z.get(n+"-"+e));
			ctrs20.get(""+e).addTerm(-1,a.get(""+e));
			ctrs21.get(""+e).addTerm(-1,a.get(""+e));
			
			ctrs22.get(""+e).addTerm(1,a.get(""+e));
			
			model.addGe(ctrs20.get(""+e),0);
			model.addGe(ctrs21.get(""+e),0);
			//model.addEq(ctrs22.get(""+e),1);// to force all techs.
		}
		
	//15. Lower bound on the number of techs:
		
		double minW = Math.ceil(totalServ/T);
		IloLinearNumExpr ctrs23 = model.linearNumExpr();
		for(int e=1;e<=m;e++) {
			ctrs23.addTerm(1, a.get(""+e));
		}
		model.addGe(ctrs23, minW);
		
	//16. Relation between a and w:
		
		Hashtable<String,IloLinearNumExpr> ctrs24 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=N;i++) {
				ctrs24.put(i+"-"+e,model.linearNumExpr());
				ctrs24.get(i+"-"+e).addTerm(M, a.get(""+e));
				ctrs24.get(i+"-"+e).addTerm(-1, w.get(i+"-"+e));
				model.addGe(ctrs24.get(i+"-"+e),0);
			}
		}
		
	//17. Relation between b and w:
		
		Hashtable<String,IloLinearNumExpr> ctrs25 = new Hashtable<String,IloLinearNumExpr>();
		
		for(int e=1;e<=m;e++) {
			for(int i=1;i<=n-2;i++) {
				ctrs25.put(i+"-"+e,model.linearNumExpr());
				ctrs25.get(i+"-"+e).addTerm(M, b.get(i+"-"+e));
				ctrs25.get(i+"-"+e).addTerm(-1, w.get(i+"-"+e));
				model.addGe(ctrs25.get(i+"-"+e),0);
			}
		}
	}
	
	/**
	 * Creates model variables
	 * @param model
	 * @param data
	 * @param graph
	 * @return
	 * @throws IloException
	 */
	public void createVariables(DataHandler data,OriginalGraph graph)throws IloException {
		
		//2. Creates hashtables to store variables:
		
		//2.1 Arcs:
		
		x = new Hashtable<String,IloNumVar>();
		h = new Hashtable<String,IloNumVar>();
		
		//2.2 Nodes:
		
		y = new Hashtable<String,IloNumVar>();
		z = new Hashtable<String,IloNumVar>();
		g = new Hashtable<String,IloNumVar>();
		w = new Hashtable<String,IloNumVar>();
		b = new Hashtable<String,IloNumVar>();
		
		//2.3 Workers:
		
		a = new Hashtable<String,IloNumVar>();
		
		//5. Creates the variables:
		
			// Arcs:
			
			for(int e=1;e<=m;e++) {
				for(int i=1;i<=N;i++) {
					for(int j=1;j<=N;j++) {
						if(i != j) {
							if(i <= n) {
								if(j <= n) {
									x.put(i+"-"+j+"-"+e, model.boolVar());
									if(wd.get(i+"-"+j) < md) {
										h.put(i+"-"+j+"-"+e, model.boolVar());
									}
								}
								if(j > n && i+n != j) {
									if(wd.get(i+"-"+j) < md) {
										h.put(i+"-"+j+"-"+e, model.boolVar());
									}
								}
							}
							if(i>n && j<=n && i!=j+n) {
								x.put(i+"-"+j+"-"+e, model.boolVar());
							}
						}
					}
				}
			}
	
		// Nodes:
			
			for(int e=1;e<=m;e++) {
				for(int i=1;i<=N;i++) {
					if(i<=n) {
						y.put(i+"-"+e,model.boolVar());
						g.put(i+"-"+e,model.boolVar());
						b.put(i+"-"+e,model.boolVar());
						
					}
					z.put(i+"-"+e,model.boolVar());
					w.put(i+"-"+e,model.numVar(0, T));
				}
				a.put(""+e, model.boolVar());
			}
		
		
	}
	
	/**
	 * Creates the main parameters for the model
	 * @param data
	 * @param graph
	 */
	public void createParameters(DataHandler data,OriginalGraph graph) {
		
		//0. Creates the parameters we will need to make this easier:
		
			//Number of workers:
		
			cv = 1;
			cf = 0;//400;
			TS = data.getlW();
			md = data.getMaxDistBTwoP();
			M = data.getlT();
			T = data.getlT();
			D = 5;
			ds = 0.5;
			ws = 0.066666;
			

		// Number of initial nodes:
		
			int nAu = graph.getNumNodes();

			//Positions and service time:
		
				posx = new Hashtable<Integer,Double>();
				posy = new Hashtable<Integer,Double>();
				serv = new Hashtable<Integer,Integer>();
				
				
				for(int i=1;i<=nAu;i++) {
					posx.put(i,data.getxCoors().get(i-1));
					posx.put(i+nAu+1,data.getxCoors().get(i-1));
					posy.put(i,data.getyCoors().get(i-1));
					posy.put(i+nAu+1,data.getyCoors().get(i-1));
				}
				posx.put(nAu+1,data.getxCoors().get(nAu-1));
				posy.put(nAu+1,data.getyCoors().get(nAu-1));
				posx.put(nAu+1+nAu+1,data.getxCoors().get(nAu-1));
				posy.put(nAu+1+nAu+1,data.getyCoors().get(nAu-1));
				
				for(int i=1;i<=nAu;i++) {
					serv.put(i,data.getServicesTimes().get(i-1));
					totalServ += serv.get(i);
				}
				for(int i=nAu+1;i<=(nAu+1*2);i++) {
					serv.put(i, 0);
				}

		m = (int) Math.ceil(totalServ/T);//5;
				
		// Additionals:
			
			l = new Hashtable<String,Integer>(); //1: If the arc between i and j exists 0: otherwise
			wt = new Hashtable<String,Double>(); //1:Walking time of arc between i and j
			wd = new Hashtable<String,Double>(); //1:Walking distance of arc between i and j
			dt = new Hashtable<String,Double>(); //1:Driving time of arc between i and j
			dd = new Hashtable<String,Double>(); //1:Driving time of arc between i and j
			
		// Final n's and n2:
			
		n = nAu+1;
		N = n*2;

		// Complete those hashtables!:
			
			for(int i=1;i<=N;i++) {
				for(int j=1;j<=N;j++) {
					if(i != j) {
						if(i <= n) {
							l.put(i+"-"+j, 1);
							dd.put(i+"-"+j,(double) Math.round(Math.sqrt(Math.pow(posx.get(i)-posx.get(j), 2) + Math.pow(posy.get(i)-posy.get(j), 2))));
							wd.put(i+"-"+j,dd.get(i+"-"+j));
							wt.put(i+"-"+j,wd.get(i+"-"+j)/ws);
							dt.put(i+"-"+j,dd.get(i+"-"+j)/ds);
						}
						if(i > n && j <= n) {
							l.put(i+"-"+j, 1);
							dd.put(i+"-"+j,(double) Math.round(Math.sqrt(Math.pow(posx.get(i)-posx.get(j), 2) + Math.pow(posy.get(i)-posy.get(j), 2))));
							wd.put(i+"-"+j,dd.get(i+"-"+j));
							wt.put(i+"-"+j,wd.get(i+"-"+j)/ws);
							dt.put(i+"-"+j,dd.get(i+"-"+j)/ds);
						}
					}
				}
			}

			
	}


	/**
	 * @return the m
	 */
	public int getNumW() {
		return m;
	}


	/**
	 * @param m the m to set
	 */
	public void setNumW(int m) {
		this.m = m;
	}


	/**
	 * @return the cv
	 */
	public double getCv() {
		return cv;
	}


	/**
	 * @param cv the cv to set
	 */
	public void setCv(double cv) {
		this.cv = cv;
	}


	/**
	 * @return the cf
	 */
	public double getCf() {
		return cf;
	}


	/**
	 * @param cf the cf to set
	 */
	public void setCf(double cf) {
		this.cf = cf;
	}


	/**
	 * @return the tS
	 */
	public double getTS() {
		return TS;
	}


	/**
	 * @param tS the tS to set
	 */
	public void setTS(double tS) {
		TS = tS;
	}


	/**
	 * @return the md
	 */
	public int getMd() {
		return md;
	}


	/**
	 * @param md the md to set
	 */
	public void setMd(int md) {
		this.md = md;
	}




	/**
	 * @return the t
	 */
	public int getT() {
		return T;
	}


	/**
	 * @param t the t to set
	 */
	public void setT(int t) {
		T = t;
	}


	/**
	 * @return the d
	 */
	public int getD() {
		return D;
	}


	/**
	 * @param d the d to set
	 */
	public void setD(int d) {
		D = d;
	}


	/**
	 * @return the ds
	 */
	public double getDs() {
		return ds;
	}


	/**
	 * @param ds the ds to set
	 */
	public void setDs(double ds) {
		this.ds = ds;
	}


	/**
	 * @return the ws
	 */
	public double getWs() {
		return ws;
	}


	/**
	 * @param ws the ws to set
	 */
	public void setWs(double ws) {
		this.ws = ws;
	}


	/**
	 * @return the m
	 */
	public int getM() {
		return M;
	}


	/**
	 * @param m the m to set
	 */
	public void setM(int m) {
		M = m;
	}
	
	public Hashtable<Integer,Object> readInitialSolutionTxt(DataHandler data, int re,int heur)throws IOException {
		
		// Path to a past solution:
		
			String ruta = "./results/Auxiliar"+"/RouteArcs"+data.getInst()+"-"+(-1)+"-"+(-1)+".txt"; //This can be generalized a little bit more.
		
		// Creates a buffered reader:

			BufferedReader buff = new BufferedReader(new FileReader(ruta));
	
		// Store arcs:
		
			ArrayList<Integer> routeIDs = new ArrayList<Integer>();
			ArrayList<Integer> tails = new ArrayList<Integer>();
			ArrayList<Integer> heads = new ArrayList<Integer>();
			ArrayList<Integer> types = new ArrayList<Integer>();
			Hashtable<Integer,ArrayList<Integer>> customers = new Hashtable<Integer,ArrayList<Integer>>();
			Hashtable<Integer,ArrayList<Integer>> parkings = new Hashtable<Integer,ArrayList<Integer>>();
			
		// Read line by line:
		
			int cantWorkers = 0;
			int routeAnt = -1;
			String line = buff.readLine();
			while(line != null) {
				String[] attrs = line.split(";");
				int tail = Integer.parseInt(attrs[0]);
				int head =Integer.parseInt(attrs[1]);
				int route = Integer.parseInt(attrs[3]);
				int type = Integer.parseInt(attrs[2]);
				
				if(route != routeAnt) {
					cantWorkers++;
					customers.put(cantWorkers, new ArrayList<Integer>());
					parkings.put(cantWorkers, new ArrayList<Integer>());
					routeAnt = route;
				}
				if(!customers.get(cantWorkers).contains(tail) && tail!=(n-1)) {
					customers.get(cantWorkers).add(tail);
				}
				if(!customers.get(cantWorkers).contains(head) && head!=(n-1)) {
					customers.get(cantWorkers).add(head);
				}
				routeIDs.add(cantWorkers);
				tails.add(tail);
				heads.add(head);
				types.add(type);
				line = buff.readLine();
							
			}
			
		//3. Builds routes..the hardest part!: Tip:read routeStrings files..
			
			String ruta2 = "./results/Auxiliar"+"/RouteStrings"+data.getInst()+"-"+(-1)+"-"+(-1)+".txt"; //This can be generalized a little bit more.
			
			// Creates a buffered reader:

			BufferedReader buff2 = new BufferedReader(new FileReader(ruta2));
				
			// Create a hashtable to store the string for each route
			
			Hashtable<Integer,String> routesStrings = new Hashtable<Integer,String>();
			
			// Read the file:
			
			line = buff2.readLine();
			cantWorkers = 0;
			while(line != null) {
				String[] attrs = line.split(";");
				cantWorkers++;
				attrs[2] = attrs[2].replaceAll(" ", "");
				routesStrings.put(cantWorkers,attrs[2]);
				line = buff2.readLine();
			}
			
			//Arcs in order:
			
			Hashtable<Integer,ArrayList<Integer>> tailsO = new Hashtable<Integer,ArrayList<Integer>>();
			Hashtable<Integer,ArrayList<Integer>> headsO = new Hashtable<Integer,ArrayList<Integer>>();
			Hashtable<Integer,ArrayList<Integer>> typesO = new Hashtable<Integer,ArrayList<Integer>>();
			
			for(int e=1;e<=cantWorkers;e++) {
				
				//Initializes the arrays:
				
				tailsO.put(e, new ArrayList<Integer>());
				headsO.put(e, new ArrayList<Integer>());
				typesO.put(e, new ArrayList<Integer>());
				
				//Recovers the actual route and splits by "|"
				
				String cadActual = routesStrings.get(e);
				String[] cadActual_s = cadActual.split("[|]");

				//The first one is always the first trip from the CD:
				
				String[] tripAct3 = cadActual_s[0].split("---");
				String[] tripAct = cadActual_s[0].split("->");
				if(tripAct.length == 3 && tripAct3.length<=1) {
					tailsO.get(e).add(n-1);
					headsO.get(e).add(Integer.parseInt(tripAct[2])+1);
					typesO.get(e).add(1);
				}else {

					tripAct = cadActual_s[0].split("->");
					String[] tripAct2 = null;
					//1. try to split by "---" to see if there's a subtour:
					if(tripAct.length==3) {
						tripAct2 = tripAct[2].split("---");
					}else {
						tripAct2 = tripAct[1].split("---");
					}
					if(tripAct2.length == 1) { //Is just a move in car!.
						
						tailsO.get(e).add(Integer.parseInt(tripAct[0])+1);
						headsO.get(e).add(Integer.parseInt(tripAct[1])+1);
						typesO.get(e).add(1);
						
					}else {
						if(tripAct.length==3) {
							tailsO.get(e).add(n-1);
							headsO.get(e).add(Integer.parseInt(tripAct2[0])+1);
							typesO.get(e).add(1);
						}
						
						if(!parkings.get(e).contains(Integer.parseInt(tripAct2[0])+1)) {
							parkings.get(e).add(Integer.parseInt(tripAct2[0])+1);
						}
						for(int j=0;j<tripAct2.length-1;j++) {
							tailsO.get(e).add(Integer.parseInt(tripAct2[j])+1);
							headsO.get(e).add(Integer.parseInt(tripAct2[j+1])+1);
							typesO.get(e).add(2);
						}
					}
				}
				
				//For the rest: first count -> and ---:
				
				for(int i=1;i<cadActual_s.length;i++) {
					if(cadActual_s[i].length() > 0) {
						
						int c1 = cadActual_s[i].split("->").length;
						if(c1 == 1) { //Subtour from the node..that can happen
							tripAct = cadActual_s[i].split("---");
							if(!parkings.get(e).contains(Integer.parseInt(tripAct[0])+1)) {
								parkings.get(e).add(Integer.parseInt(tripAct[0])+1);
							}
							for(int j=0;j<tripAct.length-1;j++) {
								tailsO.get(e).add(Integer.parseInt(tripAct[j])+1);
								headsO.get(e).add(Integer.parseInt(tripAct[j+1])+1);
								typesO.get(e).add(2);
							}
						}
						if(c1 == 2) { //1 move in car
							//Split by "->"

							tripAct = cadActual_s[i].split("->");
							String[] tripAct4 = tripAct[0].split("---");
							if(tripAct4.length>=1 && i==cadActual_s.length-1) {//La movida es al final (para llegar al CD

								if(!parkings.get(e).contains(Integer.parseInt(tripAct4[0])+1)) {
									parkings.get(e).add(Integer.parseInt(tripAct4[0])+1);
								}
								for(int j=0;j<tripAct4.length-1;j++) {
									tailsO.get(e).add(Integer.parseInt(tripAct4[j])+1);
									headsO.get(e).add(Integer.parseInt(tripAct4[j+1])+1);
									typesO.get(e).add(2);
								}

								tailsO.get(e).add(headsO.get(e).get(headsO.get(e).size()-1));
								headsO.get(e).add(n);
								typesO.get(e).add(1);
							}else {
								//1. try to split by "---" to see if there's a subtour:
								String[] tripAct2 = tripAct[1].split("---");
								if(tripAct2.length == 1) { //Is just a move in car!.
									
									tailsO.get(e).add(Integer.parseInt(tripAct[0])+1);
									headsO.get(e).add(Integer.parseInt(tripAct[1])+1);
									typesO.get(e).add(1);
									
								}else {
									
									tailsO.get(e).add(Integer.parseInt(tripAct[0])+1);
									headsO.get(e).add(Integer.parseInt(tripAct2[0])+1);
									typesO.get(e).add(1);
									if(!parkings.get(e).contains(Integer.parseInt(tripAct2[0])+1)) {
										parkings.get(e).add(Integer.parseInt(tripAct2[0])+1);
									}
									for(int j=0;j<tripAct2.length-1;j++) {
										tailsO.get(e).add(Integer.parseInt(tripAct2[j])+1);
										headsO.get(e).add(Integer.parseInt(tripAct2[j+1])+1);
										typesO.get(e).add(2);
									}
								}
							}
							
							
							
						}
						if(c1 == 3) {  //Is the end:
							tripAct = cadActual_s[i].split("->");
							
							//If ends with a subtour:
							String[] tripAct2 = tripAct[1].split("---");
							if(tripAct2.length > 1) {
								tailsO.get(e).add(Integer.parseInt(tripAct[0])+1);
								headsO.get(e).add(Integer.parseInt(tripAct2[0])+1);
								typesO.get(e).add(1);
								if(!parkings.get(e).contains(Integer.parseInt(tripAct2[0])+1)) {
									parkings.get(e).add(Integer.parseInt(tripAct2[0])+1);
								}
								for(int j=0;j<tripAct2.length-1;j++) {
									tailsO.get(e).add(Integer.parseInt(tripAct2[j])+1);
									headsO.get(e).add(Integer.parseInt(tripAct2[j+1])+1);
									typesO.get(e).add(2);
								}

								tailsO.get(e).add(headsO.get(e).get(headsO.get(e).size()-1));
								headsO.get(e).add(n);
								typesO.get(e).add(1);
							}
							else {
								tailsO.get(e).add(Integer.parseInt(tripAct[0])+1);
								headsO.get(e).add(Integer.parseInt(tripAct[1])+1);
								typesO.get(e).add(1);
								tailsO.get(e).add(Integer.parseInt(tripAct[1])+1);
								headsO.get(e).add(n);
								typesO.get(e).add(1);
							}
							
							
						}
					}
					
				}
				
				
			}

		//4. Closes the buffered reader
				
			buff.close();
			buff2.close();

		//5. Create two vectors to store variables and their initial values:
			
			int totVariables = x.size()+h.size()+y.size()+z.size()+g.size()+w.size()+b.size()+a.size();
			double[] iniValues = new double[totVariables];
			IloNumVar[] vars = new IloNumVar[totVariables];
			String[] cuales = new String[totVariables];
			
		//6. Start adding variables with their value:
			
			int contVariable = 0;
			
			//6.1 a: 1: if the worker is selected, 0:otherwise
			
			for(int e=1;e<=m;e++) {
				if(e <= cantWorkers) {
					iniValues[contVariable] = 1;
				}else {
					iniValues[contVariable] = 0;
				}
				vars[contVariable] = a.get(""+e);
				cuales[contVariable] = "a_"+e;
				contVariable++;
				
			}
			
			//6.2 b: 1: if the worker attends client i, 0:otherwise
			
			for(int e=1;e<=m;e++) {
				
				for(int i=1;i<=n;i++) {
					
					if(customers.containsKey(e)) {
						
						if(customers.get(e).contains(i)) {
							
							iniValues[contVariable] = 1;
							
						}else {
							iniValues[contVariable] = 0;
						}
					}else {
						iniValues[contVariable] = 0;
					}
					
					if(i == n && e <= cantWorkers) {
						vars[contVariable] = b.get(i+"-"+e);
						cuales[contVariable] = "b_"+i+"-"+e;
						iniValues[contVariable] = 1;
						contVariable++;
					}else {
						vars[contVariable] = b.get(i+"-"+e);
						cuales[contVariable] = "b_"+i+"-"+e;
						contVariable++;
					}
					
				}
				
			}
			
			
		//6.3 z and y: 1: if the worker begins and ends at a certain node..
			
			for(int e=1;e<=m;e++) {
				
				for(int i=1;i<=n;i++) {
					
					if(e <= cantWorkers && i==n-1) {
						iniValues[contVariable] = 1;
					}else {
						iniValues[contVariable] = 0;
					}
					
					vars[contVariable] = y.get(i+"-"+e);
					cuales[contVariable] = "y_"+i+"-"+e;
					contVariable++;
				}
				
				for(int i=1;i<=N;i++) {
					
					if(e <= cantWorkers && i==n) {
						iniValues[contVariable] = 1;
					}else {
						iniValues[contVariable] = 0;
					}
					
					vars[contVariable] = z.get(i+"-"+e);
					cuales[contVariable] = "z_"+i+"-"+e;
					contVariable++;
				}
				
			}
			
	//6.4 g 1: if the worker parks the car at a certain node..
			
			for(int e=1;e<=m;e++) {
				
				for(int i=1;i<=n;i++) {
					
					if(e <= cantWorkers && parkings.get(e).contains(i)) {
						iniValues[contVariable] = 1;
					}else {
						iniValues[contVariable] = 0;
					}
					
					vars[contVariable] = g.get(i+"-"+e);
					cuales[contVariable] = "g_"+i+"-"+e;
					contVariable++;
				}
			}
		
	//6.5 w,h and x: time of arrival at node i
			
		Hashtable<String,Double> w_sols = new Hashtable<String,Double>();
		Hashtable<String,Integer> x_sols = new Hashtable<String,Integer>();
		Hashtable<String,Integer> h_sols = new Hashtable<String,Integer>();
		Hashtable<String,Integer> numParks = new Hashtable<String,Integer>();
		
		for(int e=1;e<=m && e<=cantWorkers;e++) {
			double timeA = 0;
			//double timeS = 0;
			//double timeD = 0;
			//double timeW = 0;
			w_sols.put((n-1)+"-"+e, 0.0);
			w_sols.put((N-1)+"-"+e, 420.0);
			//Iterate through the route:
			int cArcos = tailsO.get(e).size();
			ArrayList<Integer> nodosAtendidos = new ArrayList<Integer>();
			for(int a = 0;a<cArcos; a++) {
				if(numParks.containsKey(headsO.get(e).get(a)+"-"+e)) {
					if(!nodosAtendidos.contains(tailsO.get(e).get(a))) {
						timeA += serv.get(tailsO.get(e).get(a));
						//timeS += serv.get(tailsO.get(e).get(a));
						nodosAtendidos.add(tailsO.get(e).get(a));
					}
					if(typesO.get(e).get(a) == 1) {
						timeA += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						//timeD += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						x_sols.put((tailsO.get(e).get(a))+"-"+(headsO.get(e).get(a))+"-"+e,1);
					}
					if(typesO.get(e).get(a) == 2) {
						timeA += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						//timeW += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						h_sols.put((tailsO.get(e).get(a))+"-"+((headsO.get(e).get(a))+n)+"-"+e,1);
					}
					if(w_sols.containsKey((headsO.get(e).get(a))+"-"+e)) {
						w_sols.put(((headsO.get(e).get(a))+n)+"-"+e, timeA);
					}else {
						w_sols.put((headsO.get(e).get(a))+"-"+e, timeA);
					}
				}
				else if(parkings.get(e).contains(tailsO.get(e).get(a))) {
					if(!numParks.containsKey(tailsO.get(e).get(a)+"-"+e)) {//First time in the parking
						numParks.put(tailsO.get(e).get(a)+"-"+e, 1);
						if(!nodosAtendidos.contains(tailsO.get(e).get(a))) {
							timeA += serv.get(tailsO.get(e).get(a));
							//timeS += serv.get(tailsO.get(e).get(a));
							nodosAtendidos.add(tailsO.get(e).get(a));
						}
						if(typesO.get(e).get(a) == 1) {
							timeA += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							//timeD += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							x_sols.put((tailsO.get(e).get(a))+"-"+(headsO.get(e).get(a))+"-"+e,1);
						}
						if(typesO.get(e).get(a) == 2) {
							timeA += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							//timeW += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							h_sols.put((tailsO.get(e).get(a))+"-"+(headsO.get(e).get(a))+"-"+e,1);
						}
						if(w_sols.containsKey((headsO.get(e).get(a))+"-"+e)) {
							w_sols.put(((headsO.get(e).get(a))+n)+"-"+e, timeA);
						}else {
							w_sols.put((headsO.get(e).get(a))+"-"+e, timeA);
						}
					}else {//second time in the parking
						if(!nodosAtendidos.contains(tailsO.get(e).get(a))) {
							timeA += serv.get(tailsO.get(e).get(a));
							//timeS += serv.get(tailsO.get(e).get(a));
							nodosAtendidos.add(tailsO.get(e).get(a));
						}
						if(typesO.get(e).get(a) == 1) {
							timeA += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							//timeD += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							x_sols.put(((tailsO.get(e).get(a))+n)+"-"+(headsO.get(e).get(a))+"-"+e,1);
						}
						if(typesO.get(e).get(a) == 2) {
							timeA += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							//timeW += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
							h_sols.put((tailsO.get(e).get(a))+"-"+(headsO.get(e).get(a))+"-"+e,1);
						}
						if(w_sols.containsKey((headsO.get(e).get(a))+"-"+e)) {
							w_sols.put(((headsO.get(e).get(a))+n)+"-"+e, timeA);
						}else {
							w_sols.put((headsO.get(e).get(a))+"-"+e, timeA);
						}
					}
				}else {
					if(!nodosAtendidos.contains(tailsO.get(e).get(a))) {
						timeA += serv.get(tailsO.get(e).get(a));
						//timeS += serv.get(tailsO.get(e).get(a));
						nodosAtendidos.add(tailsO.get(e).get(a));
					}
					if(typesO.get(e).get(a) == 1) {
						timeA += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						//timeD += dt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						x_sols.put(((tailsO.get(e).get(a)))+"-"+(headsO.get(e).get(a))+"-"+e,1);
					}
					if(typesO.get(e).get(a) == 2) {
						timeA += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						//timeW += wt.get((tailsO.get(e).get(a)+"-"+headsO.get(e).get(a)));
						h_sols.put((tailsO.get(e).get(a))+"-"+(headsO.get(e).get(a))+"-"+e,1);
					}
					if(w_sols.containsKey((headsO.get(e).get(a))+"-"+e)) {
						w_sols.put(((headsO.get(e).get(a))+n)+"-"+e, timeA);
					}else {
						w_sols.put((headsO.get(e).get(a))+"-"+e, timeA);
					}
				}
			}
		}
		
		
		//Add the values:
		
		for(int e=1;e<=m;e++) {
			
			for(int i=1;i<=N;i++) {
				if(e <= cantWorkers && w_sols.containsKey(i+"-"+e)) {
					iniValues[contVariable] = w_sols.get(i+"-"+e);
				}else {
					iniValues[contVariable] = 0;
				}
				vars[contVariable] = w.get(i+"-"+e);
				cuales[contVariable] = "w_"+i+"-"+e;
				contVariable++;
			}
			
			for(int i=1;i<=N;i++) {
				
				for(int j=1;j<=N;j++) {
					
					if(x.containsKey(i+"-"+j+"-"+e)) {
						
						if(x_sols.containsKey(i+"-"+j+"-"+e)) {
							iniValues[contVariable] = x_sols.get(i+"-"+j+"-"+e);
						}else {
							iniValues[contVariable] = 0;
						}
						vars[contVariable] = x.get(i+"-"+j+"-"+e);
						cuales[contVariable] = "x_"+i+"-"+j+"-"+e;
						contVariable++;
					}
					if(h.containsKey(i+"-"+j+"-"+e)) {
						
						if(h_sols.containsKey(i+"-"+j+"-"+e)) {
							iniValues[contVariable] = h_sols.get(i+"-"+j+"-"+e);
						}else {
							iniValues[contVariable] = 0;
						}
						vars[contVariable] = h.get(i+"-"+j+"-"+e);
						cuales[contVariable] = "h_"+i+"-"+j+"-"+e;
						contVariable++;
					}
				}
			}
		}
		
		/**
		System.out.println(totVariables +" - "+contVariable);
		for(int i = 0;i<totVariables;i++) {
			if(iniValues[i]>0) {
				System.out.println(cuales[i]+" - "+iniValues[i]);
			}
			
		}
		*/
		
		//for(int i=0;i<vars.)
		Hashtable<Integer,Object> rta = new Hashtable<Integer,Object>();
		rta.put(1, iniValues);
		rta.put(2, vars);
		rta.put(3, cuales);
		return(rta);
		
	}
	
	
	
}
