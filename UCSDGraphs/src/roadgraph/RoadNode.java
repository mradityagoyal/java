package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class RoadNode {

	final GeographicPoint location;
	final List<RoadEdge> outgoing;
	private double distFromStart;
	private double distToGoal;

	public RoadNode(GeographicPoint location) {
		super();
		this.location = location;
		this.outgoing = new ArrayList<>();
		this.distFromStart = Double.MAX_VALUE;
		this.distToGoal = Double.valueOf(0);
	}

	public double getDistFromStart() {
		return distFromStart;
	}

	public void setDistFromStart(double distFromStart) {
		this.distFromStart = distFromStart;
	}

	public double getDistToGoal() {
		return distToGoal;
	}

	public void setDistToGoal(double distToGoal) {
		this.distToGoal = distToGoal;
	}
	
}
