using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallPivotingAlgorithm : MonoBehaviour {
	List<Point> points;
	Front f;

	void RunBallPivot() {
		
		while (true) {
			Edge e;
			while((e = f.GetActiveEdge()) != null) {
				Point p;
				if((p = BallPivot(e)) != null && (!f.IsUsed(p) || f.OnFront(p))){
					OutputTriangle(p, e.First, e.Second);
					Join(e, p, f);
					if (f.Contains(new Edge(e.First, p)))
						Glue(new Edge(p, e.First), new Edge(e.First, p));
					if(f.Contains(new Edge(e.Second, p)))
						Glue(new Edge(p, e.First), new Edge(p, e.First));
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

	private void Join(Edge e, Point p, Front f) {
		
	}

	private void OutputTriangle(Point p1, Point p2, Point p3) {
		
	}

	private Point BallPivot(Edge e) {
		return null;
	}
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
	List<LinkedList<Edge>> edgeCollections;

	internal bool Contains(Edge edge) {
		return false;
	}

	internal Edge GetActiveEdge() {
		return null;
	}

	internal void Insert(Edge edge) {
		
	}

	internal bool IsUsed(Point p) {
		return false;
	}

	internal bool OnFront(Point p) {
		return false;
	}
}


class Helper {

}
