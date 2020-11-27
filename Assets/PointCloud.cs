using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointCloud<T>
	where T : Point {
	
	public List<T> Points { get; set; }
	public int Count { get; set; }

	public PointCloud(int count) {
		Count = count;
		Points = new List<T>(count);
	}

	public void Add(T p) {
		Points.Add(p);
	}

	public void Clear() {
		Points.Clear();
		Count = 0;
	}

	public T Get(int index) {
		if (index >= 0 && index < Count)
			return Points[index];
		else
			return null;
	}
}

public class KDTree {

}

public class Point {
	public float x { get; set; }
	public float y { get; set; }
	public float z { get; set; }

	public Point(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

public class PointNormal : Point{
	public float nx { get; set; }
	public float ny { get; set; }
	public float nz { get; set; }
	public float Curvature { get; set; }

	public PointNormal(float x, float y, float z, float nx = 0, float ny = 0, float nz = 0, float curvature = 0) 
		:base(x, y, z){
		this.nx = nx;
		this.ny = ny;
		this.nz = nz;
		this.Curvature = curvature;
	}
}