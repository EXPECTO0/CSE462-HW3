using UnityEngine;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

public class Problem2 : MonoBehaviour
{
    private List<List<Vector3>> imagePointsDatas = new List<List<Vector3>>
    {
        new List<Vector3> { new Vector3(1883, 1674, 0), new Vector3(1970, 1690, 0), new Vector3(1850, 1768, 0), new Vector3(1940, 1790, 0), new Vector3(1915, 1725, 0) },
        new List<Vector3> { new Vector3(2090, 1585, 0), new Vector3(2175, 1595, 0), new Vector3(2080, 1671, 0), new Vector3(2170, 1683, 0), new Vector3(2130, 1635, 0) },
        new List<Vector3> { new Vector3(2716, 1500, 0), new Vector3(2804, 1480, 0), new Vector3(2777, 1592, 0), new Vector3(2870, 1577, 0), new Vector3(2793, 1534, 0) },
        new List<Vector3> { new Vector3(2820, 1576, 0), new Vector3(2915, 1553, 0), new Vector3(2913, 1694, 0), new Vector3(3010, 1667, 0), new Vector3(2917, 1619, 0) },
        new List<Vector3> { new Vector3(2400, 1733, 0), new Vector3(2507, 1726, 0), new Vector3(2440, 1857, 0), new Vector3(2560, 1857, 0), new Vector3(2480, 1790, 0) }
    };

    private List<Vector3> planeWorldPoints = new List<Vector3>
    {
        new Vector3(0, 0, 10),   // Top Left
        new Vector3(10, 0, 10),  // Top Right
        new Vector3(0, 0, 0),    // Bottom Left
        new Vector3(10, 0, 0),   // Bottom Right
        new Vector3(5, 0, 5)     // Center
    };

    private const string ImagePath = "P2_Images/IMG_";
    [SerializeField] public GameObject teapotPrefab;  
    private GameObject imagePlane;

    [SerializeField] public int imageNumber_1to5 = 1;

    void Start()
    {
        imagePlane = GameObject.Find("ImagePlane");
        if (imagePlane == null)
        {
            Debug.LogError("ImagePlane not found in the scene. Add a Plane and name it 'ImagePlane'.");
            return;
        }

        List<Texture2D> images = LoadImages(5);

        Debug.Log("============ Problem 2 ============");
        
        Debug.Log($"===== IMAGE {imageNumber_1to5} =====");

        ApplyImageToPlane(images[(imageNumber_1to5-1)]);

        var homography = ComputeHomography(planeWorldPoints, imagePointsDatas[(imageNumber_1to5-1)]);
        Debug.Log($"Homography Matrix for Image {imageNumber_1to5}:{homography}");

        Vector3 teapotWorldPosition = ComputeTeapotWorldPosition(homography);
        Debug.Log($"Teapot World Position: {teapotWorldPosition}");

        PlaceTeapot(teapotWorldPosition);
    }

    private List<Texture2D> LoadImages(int numImages)
    {
        List<Texture2D> images = new List<Texture2D>();
        for (int i = 1; i <= numImages; i++)
        {
            string path = ImagePath + i;
            Texture2D image = Resources.Load<Texture2D>(path);
            if (image != null)
            {
                images.Add(image);
            }
            else
            {
                Debug.LogError($"Failed to load image at path: {path}");
            }
        }
        return images;
    }

    private void ApplyImageToPlane(Texture2D texture)
    {
        Renderer renderer = imagePlane.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.mainTexture = texture;
            Debug.Log("Image applied to Plane.");
        }
    }

    private Matrix<double> ComputeHomography(List<Vector3> planePoints, List<Vector3> imagePoints)
    {
        int n = planePoints.Count;
        var A = Matrix<double>.Build.Dense(2 * n, 9);

        for (int i = 0; i < n; i++)
        {
            double x = planePoints[i].x;
            double z = planePoints[i].z;
            double u = imagePoints[i].x;
            double v = imagePoints[i].y;

            A.SetRow(2 * i, new[] { -x, -z, -1, 0, 0, 0, x * u, z * u, u });
            A.SetRow(2 * i + 1, new[] { 0, 0, 0, -x, -z, -1, x * v, z * v, v });
        }

        var svd = A.Svd();
        var h = svd.VT.Row(8).ToArray();

        return Matrix<double>.Build.DenseOfArray(new double[,] {
            { h[0], h[1], h[2] },
            { h[3], h[4], h[5] },
            { h[6], h[7], h[8] }
        });
    }

    private Vector3 ComputeTeapotWorldPosition(Matrix<double> homography)
    {
        // Project the homography
        var point = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 1 });
        var projected = homography * point;

        // Normalize homogeneous coordinates
        Vector3 projectedVector3 = new Vector3((float)(projected[0] / projected[2]), 0, (float)(projected[1] / projected[2]));

        Debug.Log($"Original Projected Vector3: {projectedVector3}");

        // Apply scaling for X and Z
        float imageWidth = 3264f;
        float imageHeight = 2448f;
        float planeWidth = 10f;
        float planeHeight = 10f;
        
        projectedVector3.x = projectedVector3.x * planeWidth / imageWidth; 
        projectedVector3.y = 0.1f;
        projectedVector3.z = -projectedVector3.z * planeHeight / imageHeight; 

        Debug.Log($"World Position of Teapot: {projectedVector3}");

        return projectedVector3;
    }

    private void PlaceTeapot(Vector3 position)
    {
        if (teapotPrefab != null)
        {
            Instantiate(teapotPrefab, position, Quaternion.identity);
        }
        else
        {
            Debug.LogError("Teapot prefab is not assigned!");
        }
    }
} 
