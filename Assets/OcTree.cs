using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.VFX;

using renge_pcl;


/// <summary>
/// stores point clouds for efficient rendering as voxels
/// </summary>
public class OcTree<T> where T : Point {

	PointCloud<T> Cloud { get; set; }
	bool subdivided = false;
	//HashSet<OcTreeLeaf> leaves;
	private OcTreeBranch<T>[,,] branches;
	BoundingBox rootBB_;

	/// <summary>
	/// insert points into the octree
	/// </summary>
	/// <param name="positions">positional data of the point cloud</param>
	/// <param name="colors">point cloud color data</param>
	/// <param name="depth">maximum depth to place points in octree</param>
	public void SetInputCloud(PointCloud<T> cloud, int maxSaturation) {
		Cloud = cloud;
		CalculateInitialBB();

		TrySubdivide();

		for (int i = 0; i < cloud.Count; i++) {
			Point position = cloud[i];

			if (!rootBB_.Contains(position))
				continue;
			Vector3Int xyz = rootBB_.GetOctantCoords(position);

			branches[xyz.x, xyz.y, xyz.z].AddPoint(position, i, maxSaturation);
		}
	}

	public int RadiusSearch(T point, float radius, out List<int> indices) {
		indices = new List<int>();

		for (int x = 0; x < 2; ++x) {
			for (int y = 0; y < 2; ++y) {
				for (int z = 0; z < 2; ++z) {
					List<int> ind = branches[x, y, z].RadiusSearch(point, radius);
					if (ind != null) {
						indices.AddRange(ind);
					}
				}
			}
		}

		return indices.Count;
	}

	public void ClearOcTree() {
		branches = null;
		subdivided = false;
	}

	public void CalculateInitialBB() {
		Vector3 min = Cloud[0].AsVector3();
		Vector3 max = Cloud[0].AsVector3();

		for (int i = 0; i < Cloud.Count; i++) {
			var point = Cloud[i];
			if (min.x > point.x) min.x = point.x;
			if (max.x < point.x) max.x = point.x;

			if (min.y > point.y) min.y = point.y;
			if (max.y < point.y) max.y = point.y;

			if (min.z > point.z) min.z = point.z;
			if (max.z < point.z) max.z = point.z;
		}

		min.x -= 0.1f;
		min.y -= 0.1f;
		min.z -= 0.1f;
		max.x += 0.1f;
		max.y += 0.1f;
		max.z += 0.1f;

		rootBB_ = new BoundingBox(max - ((max - min) / 2), max - min);
	}

	/// <summary>
	/// try to subdivide octree. Only succeeds if not already subdivided
	/// </summary>
	public void TrySubdivide() {
		if (subdivided) return;

		subdivided = true;

		branches = new OcTreeBranch<T>[2, 2, 2];
		for (int x = 0; x < 2; ++x) {
			for (int y = 0; y < 2; ++y) {
				for (int z = 0; z < 2; ++z) {
					branches[x, y, z] = new OcTreeBranch<T>(rootBB_.GetOctantBoundingBox(x, y, z), Cloud, null, 1);
				}
			}
		}
	}

	class BoundingBox {
		public Vector3 Center { get; private set; }
		public Vector3 SideLength { get; private set; }
		public Vector3 HalfLength { get; private set; }

		public BoundingBox(Vector3 center, Vector3 sideLength) {
			this.Center = center;
			this.SideLength = sideLength;
			HalfLength = sideLength / 2;
		}

		public bool Contains(Point point) {
			if (point.x > Center.x + HalfLength.x || point.x < Center.x - HalfLength.x ||
			point.y > Center.y + HalfLength.y || point.y < Center.y - HalfLength.y ||
			point.z > Center.z + HalfLength.z || point.z < Center.z - HalfLength.z) {
				return false;
			}
			return true;
		}

		public bool Contains(Point point, float radius) {
			float x = Mathf.Max(Center.x - HalfLength.x, Mathf.Min(Center.x + HalfLength.x, point.x));
			float y = Mathf.Max(Center.y - HalfLength.y, Mathf.Min(Center.y + HalfLength.y, point.y));
			float z = Mathf.Max(Center.z - HalfLength.z, Mathf.Min(Center.z + HalfLength.z, point.z));

			float dist = Mathf.Sqrt((x - point.x) * (x - point.x) +
									(y - point.y) * (y - point.y) +
									(z - point.z) * (z - point.z));
			return dist < radius;
		}

		public Vector3Int GetOctantCoords(Point position) {
			Vector3Int res = new Vector3Int();
			res.x = (int)Math.Floor((position.x - Center.x) / HalfLength.x) + 1;
			res.y = (int)Math.Floor((position.y - Center.y) / HalfLength.y) + 1;
			res.z = (int)Math.Floor((position.z - Center.z) / HalfLength.z) + 1;
			return res;
		}

		public BoundingBox GetOctantBoundingBox(int x, int y, int z) {
			return new BoundingBox(Center + new Vector3((x * 2 - 1) * SideLength.x, (y * 2 - 1) * SideLength.y, (z * 2 - 1) * SideLength.z) / 4, HalfLength);
		}
	}

	/// <summary>
	/// internal node and leaf
	/// </summary>
	class OcTreeBranch<J> where J : Point {
		BoundingBox bb;
		int branchDepth;
		bool subdivided = false;
		OcTreeBranch<J> parent;
		List<int> indices;
		PointCloud<J> points;

		private OcTreeBranch<J>[,,] branches;

		public OcTreeBranch(BoundingBox bb, PointCloud<J> pcl, OcTreeBranch<J> parent, int depth) {
			this.bb = bb;
			this.points = pcl;
			indices = new List<int>();
			branchDepth = depth;
			this.parent = parent;
		}

		public void TrySubdivide() {
			if (subdivided) return;
			subdivided = true;

			branches = new OcTreeBranch<J>[2, 2, 2];
			for (int x = 0; x < 2; ++x) {
				for (int y = 0; y < 2; ++y) {
					for (int z = 0; z < 2; ++z) {
						branches[x, y, z] = new OcTreeBranch<J>(bb.GetOctantBoundingBox(x, y, z), points, this, branchDepth + 1);
					}
				}
			}
		}

		public void AddPoint(Point position, int index, int maxSaturation) {

			if (!subdivided)
				indices.Add(index);

			if (subdivided) {
				Vector3Int xyz = bb.GetOctantCoords(position);

				branches[xyz.x, xyz.y, xyz.z].AddPoint(position, index, maxSaturation);
			} else if (indices.Count > maxSaturation) {
				TrySubdivide();

				foreach (var item in indices) {
					Vector3Int xyz = bb.GetOctantCoords(points[item]);
					branches[xyz.x, xyz.y, xyz.z].AddPoint(points[item], item, maxSaturation);
				}

				indices = null;
			} else {
				ClearSubTrees();
			}
		}

		public List<int> RadiusSearch(J point, float radius) {
			List<int> indices = new List<int>();

			if (bb.Contains(point, radius)) {
				if (!subdivided) {
					for (int i = 0; i < this.indices.Count; i++) {
						if ((points[this.indices[i]].AsVector3() - point.AsVector3()).sqrMagnitude < radius * radius)
							indices.Add(this.indices[i]);
					}
					return indices.Count > 0 ? indices : null;
				}
				for (int x = 0; x < 2; ++x) {
					for (int y = 0; y < 2; ++y) {
						for (int z = 0; z < 2; ++z) {
							List<int> ind = branches[x, y, z].RadiusSearch(point, radius);
							if (ind != null) {
								indices.AddRange(ind);
							}
						}
					}
				}
			} else {
				return null;
			}

			return indices;
		}

		void ClearSubTrees() {
			branches = null;
			subdivided = false;
		}
	}
}