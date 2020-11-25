using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallPivotingAlgorithm : MonoBehaviour {
	Front f;
	PointCloud cloud;
	float ballRadius;
	List<Triangle> mesh;
	Pivoter pivoter;

	void RunBallPivot() {
		pivoter = new Pivoter(cloud, ballRadius);
		while (true) {
			Edge e;
			while((e = f.GetActiveEdge()) != null) {
				Point p;
				if((p = pivoter.Pivot(e)) != null && (!pivoter.IsUsed(p) || f.OnFront(p))){
					OutputTriangle(p, e.First, e.Second);
					f.JoinAndGlue(e, p, pivoter);
				} else {
					MarkAsBoundary(e);
				}
			}

			Triangle tri;
			if ((tri = FindSeedTriangle()) != null) {
				OutputTriangle(tri.First, tri.Second, tri.Third);
				f.Insert(new Edge(tri.First, tri.Second));
				f.Insert(new Edge(tri.Second, tri.Third));
				f.Insert(new Edge(tri.Third, tri.First));
			} else
				break;
		}
	}

	private Triangle FindSeedTriangle() {
		return null;
	}

	private void MarkAsBoundary(Edge e) {

	}

	private void Glue(Edge edge1, Edge edge2) {
		
	}

	private void OutputTriangle(Point p1, Point p2, Point p3) {
		
	}
}

internal class PointCloud {
}

class Triangle {
	public Point First { get; set; }
	public Point Second { get; set; }
	public Point Third { get; set; }
}

class Point {
	public float x { get; set; }
	public float y { get; set; }
	public float z { get; set; }
	public float nx { get; set; }
	public float ny { get; set; }
	public float nz { get; set; }
	public float Curvature { get; set; }

	public Point(float x, float y, float z, float nx = 0, float ny = 0, float nz = 0, float curvature = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.nx = nx;
		this.ny = ny;
		this.nz = nz;
		this.Curvature = curvature;
	}
}

class Edge {
	public Point First { get; set; }
	public Point Second { get; set; }
	public Point OppositeVertex { get; set; }
	
	public Point BallCenter { get; private set; }
	public Point MiddlePoint { get; private set; }
	public bool Active { get; set; }
	public float PivotingRadius { get; private set; }


	public Edge() {
		First = Second = OppositeVertex = BallCenter = MiddlePoint = null;
		Active = false;
		PivotingRadius = 0;
	}

	public Edge(Point first, Point second, Point opposite, Point ballCenter) {
		First = first;
		Second = second;
		OppositeVertex = opposite;
		BallCenter = ballCenter;
		MiddlePoint = new Point((First.x + Second.x) * 0.5f, (First.y + Second.y) * 0.5f, (First.z + Second.z) * 0.5f);
		Vector3 m = new Vector3(MiddlePoint.x, MiddlePoint.y, MiddlePoint.z);
		Vector3 c = new Vector3(BallCenter.x, BallCenter.y, BallCenter.z);
		PivotingRadius = (m - c).magnitude;

		Active = true;
	}
}

class Front {
	LinkedList<Edge> front;
	LinkedListNode<Edge> pos;
	SortedDictionary<int, SortedDictionary<Edge, LinkedListNode<Edge>>> points;

	public Front() {
		front = new LinkedList<Edge>();
		pos = front.First;
	}

	internal bool Contains(Edge edge) {
		return false;
	}

	internal Edge GetActiveEdge() {
		Edge e = null;
		if(front != null && front.Count > 0) {
			bool firstLoop = true;
			for (LinkedListNode<Edge> it = pos; ; it = it.Next) {
				if(it == null) {
					it = front.First;
				}
				if(!firstLoop && it == pos) {
					break;
				}
				if (it.Value.Active) {
					pos = it;
					e = it.Value;
					break;
				}
			}
		}

		return e;
	}

	internal void Insert(Edge edge) {
		front.AddLast(edge);
		AddEdgePoints(front.Last);
	}

	internal bool OnFront(Point p) {
		return false;
	}

	private LinkedListNode<Edge> IsPresent(Edge edge) {
		return null;
	}

	private void AddEdgePoints(LinkedListNode<Edge> edge) {

	}

	private void RemoveEdgePoints(Edge edge) {

	}

	internal void JoinAndGlue(Edge e, Point p, Pivoter pivoter) {
		//join
		if (f.Contains(new Edge(e.First, p)))
			Glue(new Edge(p, e.First), new Edge(e.First, p));
		if (f.Contains(new Edge(e.Second, p)))
			Glue(new Edge(p, e.First), new Edge(p, e.First));
	}
}

internal class Pivoter {
	PointCloud cloud;
	float ballRadius;
	SortedDictionary<int, bool> notUsed;
	public Pivoter(PointCloud cloud, float ballRadius) {
	}

	internal bool IsUsed(Point p) {
		throw new NotImplementedException();
	}

	internal Point Pivot(Edge e) {
		throw new NotImplementedException();
	}
}