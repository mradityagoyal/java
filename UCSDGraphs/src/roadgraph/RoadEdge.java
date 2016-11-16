package roadgraph;

import geography.GeographicPoint;

public class RoadEdge {
	final GeographicPoint start;
	final GeographicPoint end;
	final String roadName;
	final String roadType;
	final double roadLength;

	public RoadEdge(GeographicPoint start, GeographicPoint end, String roadName, String roadType, double roadLength) {
		super();
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.roadLength = roadLength;
	}
}
