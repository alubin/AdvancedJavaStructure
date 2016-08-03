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

}
