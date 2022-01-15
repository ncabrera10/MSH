package view;

import model.Solver;

/**
 * This class handles the menu for the user.
 * @author nick0
 *
 */
public class ManagerView {

	/**
	 * Constant to manage the number of data shown  in console
	 */
	
	public static final int N = 20;

	/**
	 * Manager of the view
	 */
	
	public ManagerView() {

	}
	

	/**
	 * Console's Menu
	 */
	
	public void printMenu() {
		
		System.out.println("-------------------------The vehicle routing problem with transportable resources -----------------------------");
		System.out.println("Welcome, select one of the following options:");
		System.out.println("\t 1. Use the set of instances from Coindreau et al (2019)");
		System.out.println("\t 2. Exit");
		System.out.println("Enter your choice, then press enter: (Example., 1):");

	}
	
	/**
	 * Method to print in console a message
	 * @param message to print
	 */
	public void printMessage(String mensaje) {
		System.out.println(mensaje);
	}
	
	public void printResults(Solver solver) {
		System.out.println("");
	}
}
