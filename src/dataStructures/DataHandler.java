package dataStructures;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;

/**
 * This class manages relevant information from the current instance
 *
 */
public class DataHandler {

	/**
	 * Instance number
	 */
	
	private int inst;
	
	
	/**
	 * File path
	 */
	
	private String ruta;
	
	/**
	 * Services times
	 */
	
	private ArrayList<Integer> servicesTimes;
	
	/**
	 * x coordinates
	 */
	
	private ArrayList<Double> xCoors;
	
	/**
	 * y coordinates
	 * 
	 */
	
	private ArrayList<Double> yCoors;
	
	/**
	 * Nodes / clients
	 * 
	 */
	
	private ArrayList<Integer> nodes;
	
	/**
	 * Cost per km (Driving)
	 */
	
	private int drivCost;
	
	/**
	 * Time limit for walking sub-tours:
	 */
	
	private int lW;
	
	/**
	 * Time limit for a day:
	 */
	
	private int lT;
	
	/**
	 * Walking time limit for a day:
	 */
	
	private int dT;
	
	/**
	 * Max walking distance between two points:
	 */
	
	private int maxDistBTwoP;
	
	/**
	 * Fixed cost of hiring:
	 */
	
	private int fixCost;
	
	/**
	 * Parking time
	 */
	
	private int parkTime;
	
	/**
	 * Walking speed
	 */
	
	private int walkingSpeed;
	
	/**
	 * Driving speed
	 */
	
	private int drivingSpeed;
	
	/**
	 * Subset of routes, for each node
	 */
	private Hashtable<Integer,ArrayList<Integer>> nodeRoutes;
	
	/**
	 * The route itself
	 */
	private Hashtable<Integer,String> routeStrings;
	
	/**
	 * The route costs
	 */
	private Hashtable<Integer,Double> routeCosts;
	
	/**
	 * The route times
	 */
	private Hashtable<Integer,Double> routeTimes;
	
	/**
	 * The route heuristic id
	 */
	private Hashtable<Integer,Integer> routeHeurs;
	
	/**
	 * Additional info for printing the solution nicely in R
	 */
	
	private Hashtable<Integer,ArrayList<String>> routePrints;
	
	
	/**
	 * Creates a data handler
	 * @param r path to instance files
	 * @param in instance id
	 */
	public DataHandler(String r,Integer in) {
		
		// Assigns the path to the instance files:
		
		ruta = r;
		
		// Assigns the instances that is currently solving:
		
		inst = in;
		
		// Initializes some hashtables to store important information:
		
		nodeRoutes = new Hashtable<Integer,ArrayList<Integer>>();
		routePrints = new Hashtable<Integer,ArrayList<String>>();
		routeStrings = new Hashtable<Integer,String>();
		routeCosts = new Hashtable<Integer,Double>();
		routeTimes= new Hashtable<Integer,Double>();
		routeHeurs= new Hashtable<Integer,Integer>();

	}
	
	/**
	 * This method reads the coordinates file
	 * @return
	 * @throws IOException
	 */
	
	public void readCoordinates() throws IOException {
		
		//0. Creates a buffered reader:
		
			BufferedReader buff = new BufferedReader(new FileReader(ruta+"Coordinates_"+inst+".txt"));
		
		//1. Reads the header
			
			String line = buff.readLine();
		
		//2. Initializes all list:
		
			servicesTimes = new ArrayList<Integer>();
			xCoors = new ArrayList<Double>();
			yCoors = new ArrayList<Double>();
			nodes = new ArrayList<Integer>();
		
		// 3. Reades the 2 second line .. and iterates until there are no more nodes
			
			line = buff.readLine();
			while(line != null) {
				String[] attrs = line.split(",");
				nodes.add(Integer.parseInt(attrs[0]));
				xCoors.add(Double.parseDouble(attrs[1]));
				yCoors.add(Double.parseDouble(attrs[2]));
				servicesTimes.add(Integer.parseInt(attrs[3]));
				line = buff.readLine();
				
			}
		
			
		//4. Closes the buffered reader
		
			buff.close();
		
	}
	
	/**
	 * This method reads the master file
	 * @return
	 * @throws IOException
	 */
	
	public void readMasterFile() throws IOException {
		
		//0. Creates a buffered reader:
		
			BufferedReader buff = new BufferedReader(new FileReader(ruta+"Master.txt"));
		
		//1. Reads each line 
			
			String line = buff.readLine();
			String[] attrs = line.split(",");
			
			walkingSpeed = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			drivingSpeed = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			fixCost = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			drivCost = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			dT = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			maxDistBTwoP = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			lW = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			lT = Integer.parseInt(attrs[1]);
			
			line = buff.readLine();
			attrs = line.split(",");
			
			parkTime = Integer.parseInt(attrs[1]);		
		
		//4. Closes the buffered reader
		
			buff.close();
			
	}

	
	/**
	 * @return the inst
	 */
	public int getInst() {
		return inst;
	}

	/**
	 * @param inst the inst to set
	 */
	public void setInst(int inst) {
		this.inst = inst;
	}


	/**
	 * @return the ruta
	 */
	public String getRuta() {
		return ruta;
	}

	/**
	 * @param ruta the ruta to set
	 */
	public void setRuta(String ruta) {
		this.ruta = ruta;
	}

	/**
	 * @return the servicesTimes
	 */
	public ArrayList<Integer> getServicesTimes() {
		return servicesTimes;
	}

	/**
	 * @param servicesTimes the servicesTimes to set
	 */
	public void setServicesTimes(ArrayList<Integer> servicesTimes) {
		this.servicesTimes = servicesTimes;
	}

	/**
	 * @return the xCoors
	 */
	public ArrayList<Double> getxCoors() {
		return xCoors;
	}

	/**
	 * @param xCoors the xCoors to set
	 */
	public void setxCoors(ArrayList<Double> xCoors) {
		this.xCoors = xCoors;
	}

	/**
	 * @return the yCoors
	 */
	public ArrayList<Double> getyCoors() {
		return yCoors;
	}

	/**
	 * @param yCoors the yCoors to set
	 */
	public void setyCoors(ArrayList<Double> yCoors) {
		this.yCoors = yCoors;
	}

	/**
	 * @return the nodes
	 */
	public ArrayList<Integer> getNodes() {
		return nodes;
	}

	/**
	 * @param nodes the nodes to set
	 */
	public void setNodes(ArrayList<Integer> nodes) {
		this.nodes = nodes;
	}

	/**
	 * @return the drivCost
	 */
	public int getDrivCost() {
		return drivCost;
	}

	/**
	 * @param drivCost the drivCost to set
	 */
	public void setDrivCost(int drivCost) {
		this.drivCost = drivCost;
	}

	/**
	 * @return the lW
	 */
	public int getlW() {
		return lW;
	}

	/**
	 * @param lW the lW to set
	 */
	public void setlW(int lW) {
		this.lW = lW;
	}

	/**
	 * @return the lT
	 */
	public int getlT() {
		return lT;
	}

	/**
	 * @param lT the lT to set
	 */
	public void setlT(int lT) {
		this.lT = lT;
	}

	/**
	 * @return the dT
	 */
	public int getdT() {
		return dT;
	}

	/**
	 * @param dT the dT to set
	 */
	public void setdT(int dT) {
		this.dT = dT;
	}

	/**
	 * @return the fixCost
	 */
	public int getFixCost() {
		return fixCost;
	}

	/**
	 * @param fixCost the fixCost to set
	 */
	public void setFixCost(int fixCost) {
		this.fixCost = fixCost;
	}

	/**
	 * @return the parkTime
	 */
	public int getParkTime() {
		return parkTime;
	}

	/**
	 * @param parkTime the parkTime to set
	 */
	public void setParkTime(int parkTime) {
		this.parkTime = parkTime;
	}

	/**
	 * @return the maxDistBTwoP
	 */
	public int getMaxDistBTwoP() {
		return maxDistBTwoP;
	}

	/**
	 * @param maxDistBTwoP the maxDistBTwoP to set
	 */
	public void setMaxDistBTwoP(int maxDistBTwoP) {
		this.maxDistBTwoP = maxDistBTwoP;
	}

	/**
	 * @return the walkingSpeed
	 */
	public int getWalkingSpeed() {
		return walkingSpeed;
	}

	/**
	 * @param walkingSpeed the walkingSpeed to set
	 */
	public void setWalkingSpeed(int walkingSpeed) {
		this.walkingSpeed = walkingSpeed;
	}

	/**
	 * @return the drivingSpeed
	 */
	public int getDrivingSpeed() {
		return drivingSpeed;
	}

	/**
	 * @param drivingSpeed the drivingSpeed to set
	 */
	public void setDrivingSpeed(int drivingSpeed) {
		this.drivingSpeed = drivingSpeed;
	}

	/**
	 * @return the nodeRoutes
	 */
	public Hashtable<Integer, ArrayList<Integer>> getNodeRoutes() {
		return nodeRoutes;
	}

	/**
	 * @param nodeRoutes the nodeRoutes to set
	 */
	public void setNodeRoutes(Hashtable<Integer, ArrayList<Integer>> nodeRoutes) {
		this.nodeRoutes = nodeRoutes;
	}

	/**
	 * @return the routeStrings
	 */
	public Hashtable<Integer, String> getRouteStrings() {
		return routeStrings;
	}

	/**
	 * @param routeStrings the routeStrings to set
	 */
	public void setRouteStrings(Hashtable<Integer, String> routeStrings) {
		this.routeStrings = routeStrings;
	}

	/**
	 * @return the routeCosts
	 */
	public Hashtable<Integer, Double> getRouteCosts() {
		return routeCosts;
	}

	/**
	 * @param routeCosts the routeCosts to set
	 */
	public void setRouteCosts(Hashtable<Integer, Double> routeCosts) {
		this.routeCosts = routeCosts;
	}

	/**
	 * @return the routeTimes
	 */
	public Hashtable<Integer, Double> getRouteTimes() {
		return routeTimes;
	}

	/**
	 * @param routeTimes the routeTimes to set
	 */
	public void setRouteTimes(Hashtable<Integer, Double> routeTimes) {
		this.routeTimes = routeTimes;
	}

	/**
	 * @return the routePrints
	 */
	public Hashtable<Integer, ArrayList<String>> getRoutePrints() {
		return routePrints;
	}

	/**
	 * @param routePrints the routePrints to set
	 */
	public void setRoutePrints(Hashtable<Integer, ArrayList<String>> routePrints) {
		this.routePrints = routePrints;
	}


	/**
	 * @return the routeHeurs
	 */
	public Hashtable<Integer, Integer> getRouteHeurs() {
		return routeHeurs;
	}

	/**
	 * @param routeHeurs the routeHeurs to set
	 */
	public void setRouteHeurs(Hashtable<Integer, Integer> routeHeurs) {
		this.routeHeurs = routeHeurs;
	}


	
	
}
