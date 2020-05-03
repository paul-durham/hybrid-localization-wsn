// WSNnode.java 
// support for multiple trace storage in RSSI scan
// MCS Thesis, School of Computer Science, Carleton University
// © 2019, Paul Durham

package matlabJava;

import java.util.ArrayList;
import java.util.Collections;

public class WSNnode {
	
	static ArrayList<WSNnode> wsnodes;
	static int nodeCount;
	
	// class variables
	int ndx; // index of node in unit disk info
	double xloc; // actual x location
	double yloc; // actual y location
	double p1xloc; // x location from pass 1
	double p1yloc; // y location from pass 1
	double p2xloc; // x location from pass 2
	double p2yloc; // y location from pass 2
	boolean inpass; // true if pass in progress
	
	// traces are organized as follows
	// ArrayList of 3 element arrays
	// Each 3 element array is a trace
	// Each element of array is an ArrayList of uxloc, uyloc, or rssi
	// list of traces
	ArrayList <ArrayList<Double>[]> traces;
	
	// current trace
	ArrayList<Double>[] trace;
	
	// elements of trace
	ArrayList<Double> uxlocTrace; // current uxloc trace
	ArrayList<Double> uylocTrace; // current yxloc trace
	ArrayList<Double> rssiTrace; // current rssi trace
	
	int minpt; // minimum # of points in a trace
	
	int maxUpLen; // current maximum length of up trace
	int maxDnLen; // current maximum length of down trace
	
	public static void dumpNodes()
	{
		int j = wsnodes.size();
		System.out.println("node count "+nodeCount+" size "+j);
		for (int i=0; i < j; i++) {
			WSNnode node = wsnodes.get(i);
			node.dumpNode();
		}
	}
	
	public void dumpNode()
	{		
		// count up and down traces
		int up = 0;
		int down = 0;
		int flat = 0;
		int nt = traces.size();
		for (int k = 0; k < nt; k++) {
			ArrayList<Double> xv = traces.get(k)[0];
			ArrayList<Double> yv = traces.get(k)[1];
			ArrayList<Double> rv = traces.get(k)[2];

			// slope is up if in 1st or 3rd quadrants
			// slope1 goes from -PI to PI
			float slope1 = getAngle(traces.get(k));
			float slope2 = (float) (slope1/(Math.PI/6));
			float slope = (float) Math.tan(slope1);
			
			if (slope1 == 0 || slope1 == Math.PI || slope1 == -Math.PI)
				flat++;
			if (slope1 > 0 && slope1 <= Math.PI/2 || slope1 < -Math.PI/2)
				up++;
			else
				down++;

			double rmax = Collections.max(rv);
			int imax = rv.indexOf(rmax);

			System.out.println(" slope="+slope+" slope2="+slope2+" count="+xv.size()+
					" max="+rmax+" @ "+imax);
			
			// compute angles
			//for (int l=k+1; l < nt; l++) {
				
			//}
		}
		System.out.println("node "+ndx+" traces="+nt+" up="+up+" down="+down+" flat="+flat);
		System.out.println("longest up = "+getUpDown(1)+",  longest down="+getUpDown(-1));
		System.out.println("best up = "+getBestUp()+",  best down="+getBestDown());
	}

	private static float getAngle(ArrayList<Double>[] v ) {
		ArrayList<Double> xv = v[0];
		ArrayList<Double> yv = v[1];
		
		float xb = (float) (double) xv.get(0);
		float yb = (float) (double) yv.get(0);
		float xe = (float) (double) xv.get(xv.size()-1);
		float ye = (float) (double) yv.get(yv.size()-1);
		//float ts = (ye-yb)/(xe-xb);
		float angle = (float) Math.atan2(ye-yb, xe-xb);
		
		System.out.print(xb+","+yb+" to "+xe+","+ye);
		
		return angle;
	}
	
	private static int upDown(ArrayList<Double>[] trace) {
		int up = 0;
		
		ArrayList<Double> xv = trace[0];
		ArrayList<Double> yv = trace[1];
		
		// slope is up if in 1st or 3rd quadrants
		double xb = xv.get(0);
		double yb = yv.get(0);
		double xe = xv.get(xv.size()-1);
		double ye = yv.get(yv.size()-1);
		double slope = ((ye-yb)/(xe-xb));

		if (slope > 0)
			up = 1;
		else if (slope < 0)
			up = -1;

		return up;
	}
	
	
	public static void main(String[] args) {
		System.out.println("hi there");
		
		// set up a dummy pass
		double x = 0.0;
		double y = 0.0;
		double xn = .75;
		double yn = .5;
		
		initNodes();
		
		WSNnode node = new WSNnode(1, xn, yn, 5);
		
		double theta = Math.PI/3;
		
		while (x < 1 && y < 1) {
			double ds = (x-xn)*(x-xn) + (y-yn)*(y-yn);
			double d = Math.sqrt(ds);
			if (d < .5) {
				// within range, calculate RSSI
				double r = 1/ds;
				
				node.addRSSI(x, y, r);
			}
			else {
				node.endRSSI(false);
			}
			x += .05*Math.cos(theta);
			y += .05*Math.sin(theta);
		}
		// print out trace
		ArrayList<Double>[] t = node.traces.get(0);
		
		for (int i = 0; i<t[0].size(); i++) {
			double uxloc = t[0].get(i);
			double uyloc = t[1].get(i);
			double rssi = t[2].get(i);
			System.out.println(uxloc+" "+uyloc+" "+rssi);
		}
		
		System.out.println("unbox");
		
		double[] xx = node.getUxtrace(1);
		double[] yy = node.getUxtrace(1);
		double[] rr = node.getRSSItrace(1);
		
		
		System.out.println(xx+" "+yy+" "+rr);
		
		dumpNodes();
	}

	
	public WSNnode(int ndx, double xloc, double yloc,int minpt) {
		this.ndx = ndx;
		this.xloc = xloc;
		this.yloc = yloc;
		this.minpt = minpt;
		this.maxUpLen = 0;
		this.maxDnLen = 0;

		// not a dummy
		traces = new ArrayList <ArrayList<Double>[]>();
		inpass = false; // starts out not in pass

		wsnodes.add(this);
		nodeCount++;

		System.out.println(ndx+" new WSNnode at "+xloc+" "+yloc);
	}
	
	// initialize node count to 0
	public static void initNodes() {
		wsnodes = new ArrayList<WSNnode>();
		nodeCount = 0;
	}
	
	
	// return node with given index
	public static WSNnode getnode(int ndx) {
		// java index starts at 0
		return(wsnodes.get(ndx-1));
	}
	
	// return number of traces in node
	public int getTraceCnt() {
		return traces.size();
	}
	

	// return current trace
	public ArrayList<Double>[] getTrace(){
		return trace;
	}
	
	// return size of current trace
	public int getTraceLen(){
		return trace[0].size();
	}
	
	// this unboxes the traces so Matlab can get them as array of double
	private double[] getUnbox(ArrayList<Double> t) {
		int j = t.size();
		double[] data = new double[j];
		int i = 0;
		
		for (Double f: t) {
			data[i++] = f;
		}
		
		return data;
	}
	
	// these methods are called by Matlab and start index at 1
	
	// returns array of indexes of traces matching direction
	// == 1 up, == -1 down, == 0 flat
	public int[] getAll(int dir) {
		int[] trArray;
		ArrayList<Integer> trList = new ArrayList<Integer>();
		int nup = 0;
		for (int i=0; i<traces.size(); i++) {
			// first count up
			if (upDown(traces.get(i)) == dir){
				trList.add(i+1);
			}
		}
		trArray = new int[trList.size()];
		for (int i=0; i<trList.size(); i++) {
			trArray[i] = trList.get(i);
		}
		return trArray;
	}
	

	
	// returns index of best up trace
	// best trace has highest RSSI within middle 1/3 of trace
	public int getBestUp() {
		int n = -1;
		double rmax; // max RSSI in current trace
		double rmup = Double.NEGATIVE_INFINITY; // current max RSSI for all traces
		
		for (int i=0; i<traces.size(); i++) {
			// correct direction
			if (upDown(traces.get(i)) == 1){
				// get RSSI trace
				ArrayList<Double> rv = traces.get(i)[2];
				// size of this trace
				int rvsz = rv.size();
				// get maximum RSSI
				rmax = Collections.max(rv);
				// index of maximum RSSI
				int imax = rv.indexOf(rmax);
				
				// use trace if RSSI is biggest and in middle 
				// or no trace set yet
				if (((imax >= rvsz/3.0 && imax <= rvsz*2.0/3.0) && rmax > rmup) ||
						rmup == Double.NEGATIVE_INFINITY) {
					n = i; // new best trace
					rmup = rmax; // new best RSSI
				}
			}
		}
		return n+1;
	}
	
	// returns index of best down trace
	// best trace has highest RSSI within middle 1/3 of trace
	public int getBestDown() {
		int n = -1;
		double rmax; // max RSSI in current trace
		double rmup = Double.NEGATIVE_INFINITY; // current max RSSI for all traces
		
		for (int i=0; i<traces.size(); i++) {
			// correct direction
			if (upDown(traces.get(i)) == -1){
				// get RSSI trace
				ArrayList<Double> rv = traces.get(i)[2];
				// size of this trace
				int rvsz = rv.size();
				// get maximum RSSI
				rmax = Collections.max(rv);
				// index of maximum RSSI
				int imax = rv.indexOf(rmax);
				
				// use trace if RSSI is biggest and in middle
				// or no trace set yet
				if (((imax >= rvsz/3.0 && imax <= rvsz*2.0/3.0) && rmax > rmup) ||
						rmup == Double.NEGATIVE_INFINITY) {
					n = i; // new best trace
					rmup = rmax; // new best RSSI
				}
			}
		}
		return n+1;
	}
	
	// returns index of longest up or down trace, or 0 if none exists
	// dir = true looking for longest up trace
	// dir = false looking for longest down trace
	public int getUpDown(int dir) {
		int n = -1;
		int sz = -1;
		for (int i=0; i<traces.size(); i++) {
			if (upDown(traces.get(i)) == dir){
				// correct direction
				if (traces.get(i)[0].size() > sz) {
					sz = traces.get(i)[0].size();
					n = i;
				}
			}
		}
		return n+1;
	}
	
	public double[] getUxtrace(int ndx) {
		return getUnbox(traces.get(ndx-1)[0]);
	}
	
	public double[] getUytrace(int ndx) {
		return getUnbox(traces.get(ndx-1)[1]);
	}
	
	public double[] getRSSItrace(int ndx) {
		return getUnbox(traces.get(ndx-1)[2]);
	}
	
	public double[] getTraceLim(int ndx) {
		// limits of trace are returned in 4 element array
		double[] lim = new double[4];
		ArrayList<Double> xa = traces.get(ndx-1)[0];
		ArrayList<Double> ya = traces.get(ndx-1)[1];
		int sz = xa.size();
		
		lim[0] = xa.get(0);
		lim[1] = ya.get(0);
		lim[2] = xa.get(sz-1);
		lim[3] = ya.get(sz-1);
		
		return lim;
	}
	
	
	// add x, y, RSSI data point
	// returns: 1 new trace started; 0 trace already started
	@SuppressWarnings("unchecked")
	public int addRSSI(double uxloc, double uyloc, double rssi) {
		int rc = 0;
		//System.out.println(ndx+" addRSSI() uxloc="+uxloc+" uyloc="+uyloc+" RSSI="+rssi);
		// is a pass in progress?
		if (inpass) {
			trace = traces.get(traces.size()-1);
		} else {
			// start a new pass
			trace = (ArrayList<Double>[])new ArrayList[3];
			trace[0]= uxlocTrace = new ArrayList<Double>();
			trace[1]= uylocTrace = new ArrayList<Double>();
			trace[2]= rssiTrace = new ArrayList<Double>();
			traces.add(trace);
			inpass = true;
			rc = 1;
		}
		trace[0].add(uxloc);
		trace[1].add(uyloc);
		trace[2].add(rssi);
		return(rc);
	}
	
	public int endRSSI(boolean forceClear) {
		// called when pass is finished, i.e. node is out of range
		// returns 1 : pass ended OK
		// returns 0 : pass not in progress
		// returns -1 : pass too short
		// returns -2 : pass shorter than current max
		// returns -3 : pass unbalanced

		// forceClear == true : clear this pass
		if (inpass) {
			inpass = false;
			ArrayList<Double> rv = trace[2];
			int sz = rv.size();
			if (sz < minpt || forceClear) {
				// too few points in trace - clear pass
				System.out.println(this.ndx+" trace too short "+sz+" last x,y: "+trace[0].get(sz-1)+" "+trace[2].get(sz-1));
				traces.remove(trace);

				return -1;
			}
			
			int maxLen = (upDown(trace) > 0) ? maxUpLen : maxDnLen;
			
			// save new trace only if as long as previous max length
			if (sz > 0.75 * maxLen) {
				// update max length
				if (sz > maxLen) {
					if (upDown(trace) > 0)
						maxUpLen = sz;
					else
						maxDnLen = sz;	
				}
			}
			else {
				// too short
				System.out.println(this.ndx+" trace len < current max "+sz+" last x,y: "+trace[0].get(sz-1)+" "+trace[2].get(sz-1));
				traces.remove(trace);

				return -2;
			}
			
			// check for unbalanced trace
			double rmax = Collections.max(rv);
			// index of maximum RSSI
			int imax = rv.indexOf(rmax);
			if (imax < 3 || imax > sz-3) {
				System.out.println(this.ndx+" trace unbalanced max at:"+imax+" len="+sz);
				traces.remove(trace);
				
				return -3;
			}
			
			return 1;
		}
		return 0;
	}
}
