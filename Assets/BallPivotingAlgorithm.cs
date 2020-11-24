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
			while((e = GetActiveEdge(f)) != null) {
				Point p;
				if((p = BallPivot(e)) != null && (NotUsed(p) || OnFront(p))){
					OutPutTriangle(p, e.First, e.Second);
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
				OutPutTriangle(tri.First, tri.Second, tri.Third);
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

	private void OutPutTriangle(Point p1, Point p2, Point p3) {
		
	}

	private bool OnFront(Point p) {
		return false;
	}

	private bool NotUsed(Point p) {
		return false;
	}

	private Point BallPivot(Edge e) {
		return null;
	}

	private Edge GetActiveEdge(Front f) {
		throw null;
	}
}

class Triangle {
	public Point First { get; set; }
	public Point Second { get; set; }
	public Point Third { get; set; }
}

class Point {
	Vector3 position;
}

class Edge {
	public Point First { get; set; }
	public Point Second { get; set; }

	public Edge() {

	}

	public Edge(Point first, Point second) {
		First = first;
		Second = second;
	}
}

class Front {
	List<LinkedList<Edge>> edgeCollections;

	internal bool Contains(Edge edge) {
		return false;
	}

	internal void Insert(Edge edge) {
		
	}
}