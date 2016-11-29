/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.function.Consumer;
import java.util.stream.Stream;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	// adjacency list from geographicPoint to RoadSegment
	private Map<GeographicPoint, RoadNode> nodes;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		nodes = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return nodes.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return nodes.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return nodes.values().stream().mapToInt(x -> x.outgoing.size()).sum();
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null || nodes.containsKey(location)) {
			return false;
		}
		nodes.put(location, new RoadNode(location));
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		if (from == null || to == null || roadName == null || roadType == null || length < 0 || !nodes.containsKey(from)
				|| !nodes.containsKey(to)) {
			throw new IllegalArgumentException("Invalid parameters to add edge");
		}
		// use empty geographic points list
		nodes.get(from).outgoing.add(new RoadEdge(from, to, roadName, roadType, length));
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		Queue<GeographicPoint> q = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parents = new HashMap<>();

		q.add(start);
		visited.add(start);

		while (!q.isEmpty()) {
			GeographicPoint current = q.poll();
			if (current.x == goal.x && current.y == goal.y) {
				return unwindParents(parents, start, goal);
			}
			Stream<GeographicPoint> unvisitedNeighbors = getNeighbors(current).filter(n -> !visited.contains(n));
			unvisitedNeighbors.forEach(n -> {
				visited.add(n);
				parents.put(n, current);
				q.add(n);
				nodeSearched.accept(n);
			});
		}
		return null;
	}

	public List<GeographicPoint> unwindParents(Map<GeographicPoint, GeographicPoint> parents, GeographicPoint start,
			GeographicPoint goal) {
		LinkedList<GeographicPoint> result = new LinkedList<>();
		result.addFirst(goal);
		GeographicPoint curr = goal;
		while (!curr.equals(start)) {
			GeographicPoint next = parents.get(curr);
			result.addFirst(next);
			curr = next;
		}
		return (result);
	}

	/**
	 * returns immediate neighbors of the point.
	 * 
	 * @param pt
	 * @return
	 */
	public Stream<GeographicPoint> getNeighbors(GeographicPoint pt) throws IllegalArgumentException {
		// empty list of neighbors if pt is not in map
		if (!nodes.containsKey(pt))
			throw new IllegalArgumentException("Point" + pt + " is not in map ");

		return nodes.get(pt).outgoing.stream().map(x -> x.end);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {

		Queue<RoadNode> pq = new PriorityBlockingQueue<>(10,
				(a, b) -> Double.valueOf(a.getDistFromStart()).compareTo(b.getDistFromStart()));
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parents = new HashMap<>();

		RoadNode startNode = nodes.get(start);
		startNode.setDistFromStart(Double.valueOf(0));
		pq.add(startNode);

		while (!pq.isEmpty()) {
			RoadNode current = pq.poll();
			if (!visited.contains(current.location)) {
				visited.add(current.location);
				if (current.location.x == goal.x && current.location.y == goal.y) {
					return unwindParents(parents, start, goal);
				}

				// a stream of edges starting at current , whose end points are not in visited set. 
				Stream<RoadEdge> unvisitedNeighborEdges = nodes.get(current.location).outgoing.stream()
						.filter(e -> !visited.contains(e.end));
				unvisitedNeighborEdges.forEach(edge -> {
					nodeSearched.accept(edge.end);
					RoadNode neighbor = nodes.get(edge.end);
					// if path through current to n is shorter.
					Double distThruCurrent = current.getDistFromStart() + edge.roadLength;
					if (distThruCurrent < neighbor.getDistFromStart()) {
						neighbor.setDistFromStart(distThruCurrent);
						parents.put(neighbor.location, current.location);
						pq.add(neighbor);
					}
				});
			}

		}
		return null;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Here are some test cases you should try before you attempt the Week 3
		 * End of Week Quiz, EVEN IF you score 100% on the programming
		 * assignment.
		 */
		/*
		 * MapGraph simpleTestMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/testdata/simpletest.map",
		 * simpleTestMap);
		 * 
		 * GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		 * GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		 * 
		 * System.out.
		 * println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5"
		 * ); List<GeographicPoint> testroute =
		 * simpleTestMap.dijkstra(testStart,testEnd); List<GeographicPoint>
		 * testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * MapGraph testMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		 * 
		 * // A very simple test using real data testStart = new
		 * GeographicPoint(32.869423, -117.220917); testEnd = new
		 * GeographicPoint(32.869255, -117.216927); System.out.
		 * println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5"
		 * ); testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * // A slightly more complex test using real data testStart = new
		 * GeographicPoint(32.8674388, -117.2190213); testEnd = new
		 * GeographicPoint(32.8697828, -117.2244506); System.out.
		 * println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10"
		 * ); testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 */

		/* Use this code in Week 3 End of Week Quiz */
		/*
		 * MapGraph theMap = new MapGraph();
		 * System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772,
		 * -117.2254046); GeographicPoint end = new GeographicPoint(32.8660691,
		 * -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

}
