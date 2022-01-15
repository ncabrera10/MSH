package dataStructures;

/**
 * This class represents an arc of the original graph.
 */

public class OriginalArc {
	/**
	 * The arc's tail
	 */
	private int tail;
	/**
	 * The arc's head
	 */
	private int head;
	/**
	 * The arc's driving distance
	 */
	private double drDis;
	
	/**
	 * This method creates an arc
	 * @param t tail
	 * @param h head
	 * @param ddis distance
	 */
	public OriginalArc(int t, int h, double ddis) {
		tail = t;
		head = h;
		drDis = ddis;
	}
	
	/**
	 * @return the tail
	 */
	public int getTail() {
		return tail;
	}

	/**
	 * @param tail the tail to set
	 */
	public void setTail(int tail) {
		this.tail = tail;
	}

	/**
	 * @return the head
	 */
	public int getHead() {
		return head;
	}

	/**
	 * @param head the head to set
	 */
	public void setHead(int head) {
		this.head = head;
	}

	/**
	 * @return the drDis
	 */
	public double getDrDis() {
		return drDis;
	}

	/**
	 * @param drDis the drDis to set
	 */
	public void setDrDis(double drDis) {
		this.drDis = drDis;
	}
	
}
