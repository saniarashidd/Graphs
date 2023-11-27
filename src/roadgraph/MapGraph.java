/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

// TODO: Auto-generated Javadoc
/**
 * The Class MapGraph.
 *
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 */
public class MapGraph {
	
	/** The num vertices. */
	private int numVertices;
	
	/** The num edges. */
	private int numEdges;
	
	/** The vertices. */
	private HashMap<GeographicPoint,MapNode> vertices;
	
	/** The infin. */
	private static Double INFIN = Double.POSITIVE_INFINITY;
	
	/**
	 *  
	 * Create a new empty MapGraph.
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph.
	 *
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method
		return vertices.keySet();
		
	}
	
	/**
	 * Get the number of road segments in the graph.
	 *
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method.
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (ONLY if the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method
		
		if(vertices.containsKey(location) || location == null) {
			return false;
		}
		MapNode node = new MapNode();
		vertices.put(location, node);
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2. This will perform the required 
	 * error checking, get the MapNode associated with the "from" point and call the
	 * addEdge() method of the MapNode instance.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(from == null || to == null || roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException();
		}
		else if (!vertices.containsKey(from) || !vertices.containsKey(to)) {
			throw new IllegalArgumentException();
		}
		//TODO: Implement this method
		
		vertices.get(from).addEdge(to, roadName, roadType, length);
		
		numEdges++;
	}
	
	/** Find the path from start to goal using breadth first search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/**
	 *  Find the path from start to goal using breadth first search - Called by MapApp
	 *  Use of helper methods is encouraged.
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method and any helper methods.
		
		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		
		
		queue.add(start);
		visited.add(start);
		
		while(!queue.isEmpty()) {
			GeographicPoint next = queue.poll();
			
			nodeSearched.accept(next);
			
			if(next.equals(goal)) {
				return path(start, goal, parent);
			}
			
			for(GeographicPoint neighbor : vertices.get(next).getNeighborPoints()) {
				if(!visited.contains(neighbor)) {
					queue.add(neighbor);
					visited.add(neighbor);
					parent.put(neighbor, next);
				}
			}
		}
		return null;
	}
	
	
	
	/**
	 * Path list.
	 *
	 * @param start the start
	 * @param goal the goal
	 * @param parent the parent
	 * @return the list
	 */
	public List<GeographicPoint> path(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parent) {
		List<GeographicPoint> listToReturn = new LinkedList<GeographicPoint>();
		GeographicPoint next = goal;
		if(next.equals(start)) {
			return null;
		}
		while(next != null) {
			listToReturn.add(next);
			if(next.equals(start)) {
				break;
			}
			next = parent.get(next);
		}
		Collections.reverse(listToReturn);
		return listToReturn;
	}
	
	// DO NOT CODE ANYTHING BELOW THIS LINE UNTIL PART2

	/** Find the path from start to goal using Dijkstra's algorithm - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/**
	 *  Find the path from start to goal using Dijkstra's algorithm - Called by MapApp
	 *  Use of helper methods is encouraged.
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method with Part 2

		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		//nodeSearched.accept(next.getLocation());
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		Queue<QueueElement> pQueue = new PriorityQueue<QueueElement>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, Double> distToNode = new HashMap<GeographicPoint, Double>();
		Set<GeographicPoint> geoPoints = getVertices();
		for(GeographicPoint node : geoPoints) {
			distToNode.put(node, INFIN);
		}
		distToNode.replace(start, 0.0);
		pQueue.add(new QueueElement(start, 0.0));
		while(!pQueue.isEmpty()) {
			QueueElement curr = pQueue.poll();
			GeographicPoint currPoint = curr.getPoint();
			if(!visited.contains(currPoint)) {
				visited.add(currPoint);
				nodeSearched.accept(currPoint);
				
				if(currPoint.equals(goal)) {
					break;
				}
				enqueue(currPoint, visited, distToNode, parent, pQueue);
			}
		}
		return path(start, goal, parent);
		
		
	}
	
	/**
	 * Enqueue.
	 *
	 * @param currPt the curr pt
	 * @param visited the visited
	 * @param distToNode the dist to node
	 * @param parent the parent
	 * @param pQueue the queue
	 */
	public void enqueue(GeographicPoint currPt, Set<GeographicPoint> visited, 
			HashMap<GeographicPoint, Double> distToNode, HashMap<GeographicPoint, GeographicPoint> parent,
			Queue<QueueElement> pQueue) {
		MapNode node = vertices.get(currPt);
		double distToNext;
		if(node.getNeighborPoints() == null || node.getNeighborPoints().size() == 0) {
			return;
		}
		
		for(GeographicPoint next : node.getNeighborPoints()) {
			if(!visited.contains(next)) {
				distToNext = distToNode.get(currPt) + node.getEdge(next).getRoadLength();
				if(distToNext < distToNode.get(next)) {
					parent.put(next, currPt);
					pQueue.add(new QueueElement(next, distToNext));
					distToNode.replace(next, distToNext);
				}
			}
		}
	}

	/** Find the path from start to goal using A-Star search - without MapApp.
	 *  Calls the MapApp version to actually execute the search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/**
	 *  Find the path from start to goal using A-Star search - Called by MapApp
	 *  Use of helper methods is encouraged.
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method with Part 2
		
		// Hook for visualization.  See writeup. Note that this may actually need to be
		// located in a helper method.
		//nodeSearched.accept(next.getLocation());
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		Queue<QueueElement> pQueue = new PriorityQueue<QueueElement>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, Double> distPriority = new HashMap<GeographicPoint, Double>();
		Set<GeographicPoint> geoPoints = getVertices();
		for(GeographicPoint node : geoPoints) {
			distPriority.put(node, INFIN);
		}
		
		distPriority.replace(start, start.distance(goal));
		pQueue.add(new QueueElement(start, 0.0,goal));
		while(!pQueue.isEmpty()) {
			QueueElement curr = pQueue.poll();
			GeographicPoint currPoint = curr.getPoint();
			
			if(!visited.contains(currPoint)) {
				visited.add(currPoint);
				nodeSearched.accept(currPoint);
				
				if(currPoint.equals(goal)) {
					break;
				}
				aStarEnqueue(curr, goal, currPoint, visited, distPriority, parent, pQueue);
			}
		}
		
		return path(start, goal, parent);
		
	}

	/**
	 * A star enqueue.
	 */
	public void aStarEnqueue(QueueElement curr, GeographicPoint goal, GeographicPoint currPt, Set<GeographicPoint> visited, 
			HashMap<GeographicPoint, Double> distPriority, HashMap<GeographicPoint, GeographicPoint> parent,
			Queue<QueueElement> pQueue) {
		
		MapNode node = vertices.get(currPt);
		Double distToNext;
		Double priorityPathDistance;
		
		if(node.getNeighborPoints() == null || node.getNeighborPoints().size() == 0) {
			return;
		}
		
		Double distFromStart = curr.getDistFromStart();
		
		for(GeographicPoint next : node.getNeighborPoints()) {
			
			if(!visited.contains(next)) {
				distToNext = distFromStart + node.getEdge(next).getRoadLength();
				priorityPathDistance = distToNext + next.distance(goal);
				
				if(priorityPathDistance < distPriority.get(next)) {
					parent.put(next, currPt);
					pQueue.add(new QueueElement(next, distToNext,goal));
					distPriority.replace(next, priorityPathDistance);
				}
			}
		}
		
	}
	
	/**
	 * The main method.
	 *
	 * @param args the arguments
	 */
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should visit 13 nodes and AStar should visit 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should visit 37 nodes and AStar should visit 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
	}
	
}
