package sensitivity;

import java.io.IOException;

import globalParameters.GlobalParametersReader;
import model.Manager;

/**
 * This class is used for running several instances..mostly for testing. Without answering all the questions.
 * @author nick0
 *
 */
public class Main_MA_local {

	/**
	 * This is the main
	 * @param args
	 * @throws InterruptedException 
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException, InterruptedException {
		
		//Create the manager's
			
		Manager model = new Manager();
		
		// Recovers the main parameters for the run:
		
		int ins = Integer.parseInt(args[0]);
		int it = Integer.parseInt(args[1]);
		int ti = Integer.parseInt(args[2]);
		int rep = Integer.parseInt(args[3]);
		int fac = Integer.parseInt(args[4]);
		int heur = Integer.parseInt(args[5]);
		int ticp = Integer.parseInt(args[6]);

		//Initializes the config file:
		
		GlobalParametersReader.initialize("./config/parametersGlobal.xml");
		
		//Runs the MA:
		
		model.runMA(ins, it, ti,true,rep,fac,ticp,heur);

	}
	
}
