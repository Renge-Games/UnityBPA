using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[RequireComponent(typeof(ParticleSystem))]
public class VoxelRenderer : MonoBehaviour {
    ParticleSystem system;
    ParticleSystem.Particle[] voxels;
    bool voxelsUpdated = false;

    public float voxelScale = 0.1f;
    public float scale = 1f;

    private void Awake() {
        system = GetComponent<ParticleSystem>();
    }

    public void SetFromPointCloud(renge_pcl.PointCloud<renge_pcl.PointNormal> pcl)  {

        Vector3[] positions = new Vector3[pcl.Count];
        Color[] colors = new Color[pcl.Count];

        for (int i = 0; i < pcl.Count; i++) {
            positions[i] = pcl[i].AsVector3();
            colors[i] = new Color(Random.value, Random.value, Random.value);
        }
        SetVoxels(positions, colors);
    }

    public void SetFromASCFile(string filename) {
		List<Vector3> positions = new List<Vector3>();
		List<Color> colors = new List<Color>();

		using (var sr = new StreamReader(filename)) {
			while (!sr.EndOfStream) {
				string[] line = sr.ReadLine().Split(' ');
				positions.Add(new Vector3(float.Parse(line[0]), float.Parse(line[1]), float.Parse(line[2])));
				colors.Add(new Color(float.Parse(line[3]), float.Parse(line[4]), float.Parse(line[5])));
			}
		}
        SetVoxels(positions.ToArray(), colors.ToArray());
	}

    private void Update() {
        if (voxelsUpdated) {
            system.SetParticles(voxels, voxels.Length);
            voxelsUpdated = false;
        }
    }

    public void SetVoxels(Vector3[] positions, Color[] colors) {
        voxels = new ParticleSystem.Particle[positions.Length];
        system.maxParticles = positions.Length;
        system.Emit(positions.Length);
        system.GetParticles(voxels);

        for (int i = 0; i < positions.Length; i++) {
            voxels[i].position = positions[i] * scale;
            voxels[i].startColor = colors[i];
            voxels[i].startSize = voxelScale;
        }

        Debug.Log("Voxels set! Voxel count: " + voxels.Length);
        voxelsUpdated = true;
    }

    private void OnParticleCollision(GameObject other) {
        Debug.Log("Particle collided");
    }


}
