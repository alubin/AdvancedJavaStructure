/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	//This 
	private Map<GeographicPoint,ArrayList<GeographicPoint>> mapGrpList;


	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		mapGrpList = new HashMap<GeographicPoint, ArrayList<GeographicPoint>>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		int size = mapGrpList.size();
		return size;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> vertices = mapGrpList.keySet();
		return vertices;
	}


	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		int edges = 0; 
		for(Entry<GeographicPoint, ArrayList<GeographicPoint>> entry  : mapGrpList.entrySet())
		{
			edges += entry.getValue().size();
		}
		return edges;
	}



	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		boolean result = false;
		if(location != null)
		{
			if(!mapGrpList.containsKey(location))
			{
				mapGrpList.put(location, new ArrayList<GeographicPoint>());
				result = true;
			}
		}
		return result;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
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

		//TODO: Implement this method in WEEK 2
		if(from.distance(to) < 0)
		{
			throw new IllegalArgumentException();
		}
		if(from == null || to == null || roadName == null || roadType == null)
		{
			throw new IllegalArgumentException();
		}
		if(!mapGrpList.containsKey(from) || !mapGrpList.containsKey(to))
		{
			throw new IllegalArgumentException();
		}

		//Adds to the list of neighbors that from now knows about.
		ArrayList <GeographicPoint> values = mapGrpList.get(from);
		//Adds Value to the list of neighbors for from. Since this is a directed graph, only one direction is needed.
		values.add(to);
		//Put the new values back into the Adjancey List for the graph.
		mapGrpList.put(from, values);

		//		printGraph();
		//		System.out.println("Map Stuff " + mapGrpList.toString());


	}


	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
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
		List<GeographicPoint> nodeQueue = new LinkedList<GeographicPoint>();
		List<GeographicPoint> returnList = null;
		List<GeographicPoint> visitedNode = new ArrayList<GeographicPoint>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();

		nodeQueue.add(start);

		while(!nodeQueue.isEmpty())
		{
			//Get the first element of the queue to check for its children
			GeographicPoint node = (GeographicPoint) nodeQueue.remove(0);
			
			// Hook for visualization.
			nodeSearched.accept(node);

			//If the node is the goal is found, do something.
			if(node.equals(goal))
			{
				//Populate Return List
				returnList = backTrack(parentMap, start, goal);
			}

			for(GeographicPoint point: mapGrpList.get(node))
			{
				if(point != null)
				{

					if(!visitedNode.contains(point))
					{

						visitedNode.add(point);
						nodeQueue.add(point);
						parentMap.put(point, node);


					}
				}
			}


		}

		return returnList;
	}


	/**
	 * This methods reverse the order of parents from the end to the beginning.
	 * @param parent
	 * @param start
	 * @param goal
	 * @return
	 */
	private List<GeographicPoint> backTrack(Map<GeographicPoint, GeographicPoint> parent, GeographicPoint start, GeographicPoint goal)
	{
		List<GeographicPoint> outputList = new ArrayList< GeographicPoint>();
		GeographicPoint index = parent.get(goal);

		//Find every parent, until we hit the top.
		while(index != start)
		{
			outputList.add(0, index);
			index = parent.get(index);
			System.out.println("Output List = " + outputList);
		}

		//Add Start to the beginning
		outputList.add(0,start);

		//Adding the goal to the end of the list
		outputList.add(goal);

		return outputList;

	}


	/** Find the path from start to goal using Dijkstra's algorithm
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

	/** Find the path from start to goal using Dijkstra's algorithm
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
		// TODO: Implement this method in WEEK 3
		Comparator<GeographicPoint> pointComparator = new Comparator<GeographicPoint>()
		{

			@Override
			public int compare(GeographicPoint o1, GeographicPoint o2) 
			{
				return (int) (o1.distance(o2) -  o2.distance(o1));
			}

		};
		PriorityQueue<GeographicPoint> unVisited = new PriorityQueue<GeographicPoint>(11, pointComparator);
		List<GeographicPoint> visitedNode = new LinkedList<GeographicPoint>();
		List<GeographicPoint> returnList = null;
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		Map<MapEdge, Double> costMap = new HashMap<MapEdge, Double>();



		unVisited.add(start);

		while(!unVisited.isEmpty())
		{
			int cost = Integer.MAX_VALUE;
			//Get the first element of the queue to check for its children
			GeographicPoint node = (GeographicPoint) unVisited.peek();

			//If the node is the goal is found, do something.
			if(node.equals(goal) && calculateCost(parentMap, start, goal) < cost)
			{
				System.out.println("Goal Found!");
				//Populate Return List
				returnList = backTrack(parentMap, start, goal);
				cost = calculateCost(parentMap, start, goal);
				System.out.println("Cost = " + cost);
			}
			else if(node.equals(goal))
			{
				System.out.println("Goal was found, but not the best path.");
			}

			//Check the neighbors of the node.
			for(GeographicPoint point: mapGrpList.get(node))
			{
				if(point != null)
				{

					if(!visitedNode.contains(point))
					{

						//Determine the cost of the edges
						MapEdge edge = new MapEdge(node, point);
//						visitedNode.add(point);
//						unVisited.add(point);
						parentMap.put(point, node);
						//If map does not contain the key, there the cost was not stored and therefore it is assumed to be infinity
						if(!costMap.containsKey(edge))
						{
							//Calculate cost of the edge and store value.
							costMap.put(edge, edge.getDistance());
							System.out.println("Cost of " + edge +" = "  + costMap.values());
						}
						


					}
				}
			}
			
			unVisited.remove();
		}


		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return returnList;
	}
	
	/**
	 * Method to calculate the cost of the path
	 */
	private final int calculateCost(Map<GeographicPoint, GeographicPoint> parent, GeographicPoint start, GeographicPoint goal)
	{
		int cost = 0;
		GeographicPoint index = parent.get(goal);
		
		//Find every parent, until we hit the top.
		while(index != start)
		{
			GeographicPoint prev = index;
			index = parent.get(index);
			cost += prev.distance(index);
		}
		
		return cost;
		
	}

	/** Find the path from start to goal using A-Star search
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

	/** Find the path from start to goal using A-Star search
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
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args)
	{
		//		printGraph();
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		// You can use this method for testing.  

		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		 */

	}

}
