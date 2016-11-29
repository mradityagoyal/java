package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class RoadNode {

	final GeographicPoint location;
	final List<RoadEdge> outgoing;
	private double distFromStart;

	public RoadNode(GeographicPoint location) {
		super();
		this.location = location;
		this.outgoing = new ArrayList<>();
		this.distFromStart = Double.MAX_VALUE;
	}

	public double getDistFromStart() {
		return distFromStart;
	}

	public void setDistFromStart(double distFromStart) {
		this.distFromStart = distFromStart;
	}

	public double getEstimatedDistToGoal(GeographicPoint goal) {
		return goal.distance(location);
	}

	
}
