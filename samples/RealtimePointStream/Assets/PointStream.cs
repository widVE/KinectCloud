using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Http;
using System;
using System.Threading.Tasks;
using System.Text;
using System.IO;
using UnityEngine.Networking;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class PointStream : MonoBehaviour
{
    Mesh _m;
    MeshFilter _mf;

    UnityWebRequestAsyncOperation responseResult = null;

    public string URI = "http://localhost:5687/frame/latest";

    void Start() {
        _mf = GetComponent<MeshFilter>();
    }

    void Update() {
        if (responseResult == null) {
            responseResult = UnityWebRequest.Get(URI).SendWebRequest();
        }
        if (responseResult != null && responseResult.isDone) {
            byte[] data = responseResult.webRequest.downloadHandler.data;

            int numPoints = (int)BitConverter.ToUInt64(data, 0);
            //if (numPoints > pointLimit) numPoints = pointLimit;

            if (numPoints >= 0) {

                Vector3[] vertices = new Vector3[numPoints];

                Color[] colors = new Color[numPoints];

                int[] indices = new int[numPoints];

                for (int i = 0; i < numPoints; i++) {
                    vertices[i] = new Vector3(
                        BitConverter.ToInt16(data, 8 + i * 9) * 0.001f,
                        BitConverter.ToInt16(data, 8 + i * 9 + 2) * -0.001f,
                        BitConverter.ToInt16(data, 8 + i * 9 + 4) * 0.001f
                    );
                    colors[i] = new Color(
                        data[8 + i * 9 + 8] / 255.0f,
                        data[8 + i * 9 + 7] / 255.0f,
                        data[8 + i * 9 + 6] / 255.0f
                    );
                    indices[i] = i;
                }

                Mesh m = new Mesh();
                m.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
                m.SetVertices(vertices);
                m.SetColors(colors);
                m.SetIndices(indices, MeshTopology.Points, 0);
                m.RecalculateBounds();

                _mf.mesh = m;
            }

            responseResult = null;
        }
    }
}
