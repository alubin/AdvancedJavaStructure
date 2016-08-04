package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private final GeographicPoint p1;
	private final GeographicPoint p2;
	
	public MapEdge(GeographicPoint point1, GeographicPoint point2)
	{
		p1 = point1;
		p2 = point2;
	}
	
	/**
	 * The cost/distance of the vertices near each other.
	 * @return
	 */
	public double getDistance()
	{
		return p1.distance(p2);
	}
	
	public GeographicPoint getVertice1()
	{
		return p1;
	}
	
	public GeographicPoint getVertice2()
	{
		return p2;
	}
	
	public String toString()
	{
		return "Point 1 = " + p1 + " Point 2 = " + p2;
	}

}
