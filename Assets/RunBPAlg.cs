﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RunBPAlg : MonoBehaviour {
	public GameObject BPAlgObject;

	void Start() {
		BallPivotingAlgorithm bpa;
		if(BPAlgObject.TryGetComponent(out bpa)) {
			Debug.Log("Running first BP Alg...");
			float[] passes = new float[] { 0.2f, 0.3f, 0.5f };
			bpa.Run(20000, 10.0f, passes);
		}
	}
}
