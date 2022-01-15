package dataStructures;

import java.util.ArrayList;

/**
 * This class represents a node of the original graph.
 */

public class OriginalNode {

	/**
	 * Node id
	 */
	private int id;
	
	/**
	 * Minimum distance to other nodes
	 */
	
	private double dmin;
	
	/**
	 * Maximum distance to other nodes
	 */
	
	private double dmax;
	
	/**
	 * Minimum delta
	 */
	
	private double minDelta;
	
	/**
	 * Head id's of outgoing arcs
	 */
	
	private ArrayList<OriginalArc> forwardStar;
	
	/**
	 * This method creates a node 
	 * @param i node id
	 */
	public OriginalNode(int i){
		forwardStar = new ArrayList<OriginalArc>();
		id = i;
		dmin = 999999;
		dmax = 0.0;
		minDelta = 999999;
	}

	/**
	 * @return the id
	 */
	public int getId() {
		return id;
	}

	/**
	 * @param id the id to set
	 */
	public void setId(int id) {
		this.id = id;
	}

	/**
	 * @return the dmin
	 */
	public double getDmin() {
		return dmin;
	}

	/**
	 * @param dmin the dmin to set
	 */
	public void setDmin(double dmin) {
		this.dmin = dmin;
	}

	/**
	 * @return the dmax
	 */
	public double getDmax() {
		return dmax;
	}

	/**
	 * @param dmax the dmax to set
	 */
	public void setDmax(double dmax) {
		this.dmax = dmax;
	}

	/**
	 * @return the minDelta
	 */
	public double getMinDelta() {
		return minDelta;
	}

	/**
	 * @param minDelta the minDelta to set
	 */
	public void setMinDelta(double minDelta) {
		this.minDelta = minDelta;
	}

	/**
	 * @return the forwardStar
	 */
	public ArrayList<OriginalArc> getForwardStar() {
		return forwardStar;
	}

	/**
	 * @param forwardStar the forwardStar to set
	 */
	public void setForwardStar(ArrayList<OriginalArc> forwardStar) {
		this.forwardStar = forwardStar;
	}
	
	/**
	 * Adds an outgoing arc (In order)
	 * @param a the arc
	 */
	public void addOutGoingArc(OriginalArc a){
		double cScore = a.getDrDis();
		
		boolean cond = true;
		int l = 0; //Por izquierda
		int r = forwardStar.size();
		int m = (int)((l+r)/2);
		double mVal = 0;
		if(forwardStar.size() == 0){
			forwardStar.add(a);
			return;
		}
		else if(forwardStar.size() == 1){
			mVal = forwardStar.get(m).getDrDis();
			if(cScore == mVal){
				forwardStar.add(a);
			}else{
				forwardStar.add(0, a);
			}
		}
		else{
			mVal = forwardStar.get(m).getDrDis();
		}
		while(cond){
			if(r-l>1){
				if(cScore < mVal){
					r = m;
					m = (int)((l+r)/2);
				}
				else if(cScore > mVal){
					l = m;
					m = (int)((l+r)/2);
				}else{
					forwardStar.add(m,a);
					return;
				}
				mVal = forwardStar.get(m).getDrDis();
			}
			else{
				cond = false;
				if(l == m){
					if(cScore == mVal){
						forwardStar.add(l+1,a);
					}else{
						forwardStar.add(cScore<mVal?l:l+1, a);
					}
				}else if(r == m){
					if(cScore == mVal){
						forwardStar.add(r+1,a);
					}else{
						forwardStar.add(cScore<mVal?r:Math.min(r+1,forwardStar.size()), a);
					}
				}
			}
		}
	}

	
	
	
}
