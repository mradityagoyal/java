package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class RoadNode {

	final GeographicPoint location;
	final List<RoadEdge> outgoing;

	public RoadNode(GeographicPoint location) {
		super();
		this.location = location;
		this.outgoing = new ArrayList<>();
	}
}
