package controller;

import java.io.IOException;
import java.util.InputMismatchException;
import java.util.Scanner;

import globalParameters.GlobalParametersReader;
import model.Manager;
import model.Solver;
import view.ManagerView;

/**
 * This class controlls both the view and the models. 
 * From this class the program is runned and controled. 
 */

public class Controller {
	
	/**
	 * View Object
	 */
	
	private ManagerView view;
	
	/**
	 * Model Object
	 */
	
	private Manager model;
	
	/**
	 * Constructor
	 * @throws InterruptedException 
	 * @throws IOException 
	 */
	public Controller() throws IOException, InterruptedException
	{
		view = new ManagerView();
		model = new Manager();
	}
	
	/**
	 * This method runs the program. 
	 */
	
	public void run() {
		
		// Creates a scanner:
		
		Scanner sc = new Scanner(System.in);
		
		// Initializes the option chosen by the user:
		
		int option = -1;
		
		// Initializes a boolean variable to end the app:
		
		boolean fin = false;
		
		// Initializes a boolean stating if a selection was made:
		
		boolean numeroEncontrado = false;
		
		// While still running:
		
		while(!fin)
		{
			// Print the menu:
			
			view.printMenu();
			
			// If the user puts an incorrect value:
			
				while (!numeroEncontrado){
					try {
						option = sc.nextInt();
						numeroEncontrado = true;
					} catch (InputMismatchException e) {
						System.out.println("You entered an invalid option");
						view.printMenu();
						sc = new Scanner(System.in);
					}
				} numeroEncontrado = false;
			
			// Try-catch to avoid starting again (due to small errors):
				
			try { 
				
				switch(option)
				{
					//Case when the user presses 1: Uses the set of instances of coindreau et al (2019)
				
					case 1:
						GlobalParametersReader.initialize("./config/parametersGlobal.xml");
						view.printMessage("Do you want to use the MSH or a MIP? (1:MSH 2:MIP)");
						int firstSelection = Integer.parseInt(sc.next());
						if(firstSelection == 1) {
							view.printMessage("Do you want to use the default parameters? (1:yes 2:no)");
							int defaults = Integer.parseInt(sc.next());
							if(defaults == 1) {
								view.printMessage("Choose the instance (Between 1 = 20_A_1 and 40 = 50_A_10):");
								int instance = Integer.parseInt(sc.next());
								System.out.println("Running instance "+instance+" with "+2500+" iterations and a time T of "+120+" seconds");
								System.out.println("");
								Solver ma = model.runMA(instance, 2500, 120,true,1,8,60,0);
								view.printResults(ma);
								view.printMessage("Want to check how far is this solution from being optimal? 1:Yes 2:No");
								int mip = Integer.parseInt(sc.next());
								if(mip == 1) {
									view.printMessage("Choose the time limit for the MIP in seconds: ");
									int tice = Integer.parseInt(sc.next());
									ma.MIP(instance,true,tice,true);
								}
								System.out.println("Don't forget to search for your solution at the following path: ");
								System.out.println("./results/RouteArcs"+instance+"-"+0+"-"+1+".txt");
								System.out.println("You can take the file and plot in: https://nicolascabrera.shinyapps.io/VRPTR/");
							}else {
								view.printMessage("Choose the instance (Between 1 = 20_A_1 and 40 = 50_A_10):");
								int instance = Integer.parseInt(sc.next());
								view.printMessage("Choose the number of iterations (I):");
								int numIterations = Integer.parseInt(sc.next());
								view.printMessage("Choose the time limit for the sampling phase in seconds (T):");
								int timeLimit = Integer.parseInt(sc.next());
								view.printMessage("Choose the time limit for the set partitioning in seconds (T):");
								int timeLimitCP = Integer.parseInt(sc.next());
								view.printMessage("Which TSP heuristics do you want to use? (0: All 1:RNN 2:RNI 3:RFI 4:RBI)");
								int whichHeuristic = Integer.parseInt(sc.next());
								view.printMessage("Choose the randomization factor for the TSP heuristics:");
								int fact = Integer.parseInt(sc.next());
								System.out.println("");
								System.out.println("Running instance "+instance+" with "+numIterations+" iterations and a time T of "+timeLimit);
								System.out.println("");
								Solver ma = model.runMA(instance, numIterations, timeLimit,true,1,fact,timeLimitCP,whichHeuristic);
								view.printResults(ma);
								view.printMessage("Want to check how far is this solution from being optimal? 1:Yes 2:No");
								int mip = Integer.parseInt(sc.next());
								if(mip == 1) {
									view.printMessage("Choose the time limit for the MIP in seconds: ");
									int tice = Integer.parseInt(sc.next());
									ma.MIP(instance,true,tice,true);
								}
								System.out.println("Don't forget to search for your solution at the following path: ");
								System.out.println("./results/RouteArcs"+instance+"-"+whichHeuristic+"-"+1+".txt");
								System.out.println("You can take the file and plot in: https://nicolascabrera.shinyapps.io/VRPTR/");
							}
						}else {
							view.printMessage("Choose the instance (Between 1 = 20_A_1 and 40 = 50_A_10):");
							int instance = Integer.parseInt(sc.next());
							view.printMessage("Choose the time limit for the MIP in seconds: ");
							int tice = Integer.parseInt(sc.next());
							Solver ma = model.runMIP(instance,true,tice,false);
							view.printResults(ma);
							System.out.println("Don't forget to search for your solution at the following path: ");
							System.out.println("./results/mip"+"/RouteArcs"+instance+"-"+".txt");
							System.out.println("You can take the file and plot in: https://nicolascabrera.shinyapps.io/VRPTR/");
						
						}
						fin=true;
						sc.close();
						break;
						
					
					case 2:
						fin=true;
						sc.close();
						break;
				}
				
				
			}catch(Exception e) { // If an error ocurred.
				e.printStackTrace(); System.out.println("Something happen. We recommend you to start over");
			}
		}	
	}
}
