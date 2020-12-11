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

namespace renge_pcl {

	namespace octree {

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
					Vector3 position = cloud[i].AsVector3();

					if (!rootBB_.Contains(position))
						continue;
					Vector3Int xyz = rootBB_.GetOctantCoords(position);

					branches[xyz.x, xyz.y, xyz.z].AddPoint(position, i, maxSaturation);
				}
			}

			public int RadiusSearch(Vector3 point, float radius, out List<int> indices) {
				indices = new List<int>();
				//List<KeyValuePair<int, float>> sortedIndices = new List<KeyValuePair<int, float>>();
				for (int x = 0; x < 2; ++x) {
					for (int y = 0; y < 2; ++y) {
						for (int z = 0; z < 2; ++z) {
							//List<KeyValuePair<int, float>> ind = branches[x, y, z].RadiusSearch(point, radius);
							//if (ind != null) {
							//	sortedIndices.AddRange(ind);
							//}
							List<int> ind = branches[x, y, z].RadiusSearch(point, radius);
							if (ind != null) {
								indices.AddRange(ind);
							}
						}
					}
				}
				//sortedIndices.Sort((first, next) => {
				//	return first.Value.CompareTo(next.Value);
				//});
				//for (int i = 0; i < sortedIndices.Count; i++) {
				//	indices.Add(sortedIndices[i].Key);
				//}
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

				public void AddPoint(in Vector3 position, int index, int maxSaturation) {

					if (!subdivided)
						indices.Add(index);

					if (subdivided) {
						Vector3Int xyz = bb.GetOctantCoords(position);

						branches[xyz.x, xyz.y, xyz.z].AddPoint(position, index, maxSaturation);
					} else if (indices.Count > maxSaturation) {
						TrySubdivide();

						foreach (var item in indices) {
							Vector3Int xyz = bb.GetOctantCoords(points[item].AsVector3());
							branches[xyz.x, xyz.y, xyz.z].AddPoint(points[item].AsVector3(), item, maxSaturation);
						}

						indices = null;
					} else {
						ClearSubTrees();
					}
				}

				public List<int> RadiusSearch(Vector3 point, float radius) {
					List<int> indices = new List<int>();

					if (bb.Contains(point, radius)) {
						if (!subdivided) {
							for (int i = 0; i < this.indices.Count; i++) {
								float sqrMag = (points[this.indices[i]].AsVector3() - point).sqrMagnitude;
								if (sqrMag < radius * radius)
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
						return indices;
					} else {
						return null;
					}
				}

				//public List<KeyValuePair<int, float>> RadiusSearch(Vector3 point, float radius) {
				//	List<KeyValuePair<int, float>> indices = new List<KeyValuePair<int, float>>();

				//	if (bb.Contains(point, radius)) {
				//		if (!subdivided) {
				//			for (int i = 0; i < this.indices.Count; i++) {
				//				float sqrMag = (points[this.indices[i]].AsVector3() - point).sqrMagnitude;
				//				if (sqrMag < radius * radius)
				//					indices.Add(new KeyValuePair<int, float>(this.indices[i], sqrMag));
				//			}
				//			return indices.Count > 0 ? indices : null;
				//		}
				//		for (int x = 0; x < 2; ++x) {
				//			for (int y = 0; y < 2; ++y) {
				//				for (int z = 0; z < 2; ++z) {
				//					List<KeyValuePair<int, float>> ind = branches[x, y, z].RadiusSearch(point, radius);
				//					if (ind != null) {
				//						indices.AddRange(ind);
				//					}
				//				}
				//			}
				//		}
				//		return indices;
				//	} else {
				//		return null;
				//	}
				//}

				void ClearSubTrees() {
					branches = null;
					subdivided = false;
				}
			}
		}
	}
}