using UnityEngine;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

public class Problem1 : MonoBehaviour
{
    void Start()
    {
        // List of all image points (Manually calculated)
        List<List<Vector2>> imagePointsList = new List<List<Vector2>>
        {
            new List<Vector2> // Image 1
            {
                new Vector2(470, 515),      // Top Left
                new Vector2(2090, 535),     // Top Right
                new Vector2(447, 2617),     // Bottom Left
                new Vector2(2074, 2623),    // Bottom Right
                new Vector2(1270, 1570)     // Center
            },
            new List<Vector2> // Image 2
            {
                new Vector2(280, 505),
                new Vector2(1950, 445),
                new Vector2(250, 2610),
                new Vector2(1925, 2695),
                new Vector2(1075, 1560)
            },
            new List<Vector2> // Image 3
            {
                new Vector2(490, 595),
                new Vector2(2150, 720),
                new Vector2(485, 2680),
                new Vector2(1945, 2620),
                new Vector2(1300, 1725)
            }
        };
    
        // Scene points (Used provided coordinate system)
        List<Vector2> scenePoints = new List<Vector2>
        {
            new Vector2(0, 0),
            new Vector2(0, 700),
            new Vector2(900, 0),
            new Vector2(900, 700),
            new Vector2(450, 350)
        };

        List<Vector2> specificScenePoints = new List<Vector2>
        {
            new Vector2(7.5f, 5.5f),
            new Vector2(6.3f, 3.3f),
            new Vector2(0.1f, 0.1f)
        };

        List<Vector2> specificImagePoints = new List<Vector2>
        {
            new Vector2(500, 400),
            new Vector2(86, 167),
            new Vector2(10, 10)
        };

        Debug.Log("============ Problem 1 ============");
        for (int i = 0; i < imagePointsList.Count; i++)
        {
            Debug.Log($"==== IMAGE {i + 1} ====");

            // --- Part 1.1: Compute Homography Matrix ---
            Debug.Log("==== Part 1.1: Homography Matrix ====");
            var homography = ComputeHomography(scenePoints, imagePointsList[i]);
            Debug.Log($"Homography Matrix for Image {i + 1}:\n{homography}");

        // I implemented the part 1.2 function but I couldn't use it. Because I didn't understand how to defiine similarityMatrix.
            // --- Part 1.2: Compute Homography with Similarity Matrix ---
            // Debug.Log("==== Part 1.2: Homography with Similarity ====");
            // var homographyWithSim = ComputeHomographyWithSimilarity(scenePoints, imagePoints, similarityMatrix);
            // Debug.Log("Homography with Similarity:\n" + homographyWithSim.ToString());

            // --- Part 1.3: Project Scene Points to Image ---
            Debug.Log("==== Part 1.3: Project Scene Points to Image ====");
            foreach (var scenePoint in scenePoints)
            {
                var projected = ProjectSceneToImage(scenePoint, homography);
                Debug.Log($"Scene Point {scenePoint} -> Projected Image Point: {projected}");
            }

            // --- Part 1.4: Project Image Points to Scene ---
            Debug.Log("==== Part 1.4: Project Image Points to Scene ====");
            foreach (var imagePoint in imagePointsList[i])
            {
                var projected = ProjectImageToScene(imagePoint, homography);
                Debug.Log($"Image Point {imagePoint} -> Projected Scene Point: {projected}");
            }

            // --- Part 1.5: Compute Errors for Each Image ---
            Debug.Log("==== Part 1.5: Projection Errors ====");
            for (int j = 0; j < scenePoints.Count; j++)
            {
                var projected = ProjectSceneToImage(scenePoints[j], homography);
                float error = Vector2.Distance(projected, imagePointsList[i][j]);
                Debug.Log($"Scene Point {scenePoints[j]} -> Projected {projected} | Image Point: {imagePointsList[i][j]} | Error: {error}");
            }

            // --- Part 1.6: Project Specific Scene Points to Image ---
            Debug.Log("==== Part 1.6: Specific Points Projections ====");
            foreach (var scenePoint in specificScenePoints)
            {
                var projected = ProjectSceneToImage(scenePoint, homography);
                Debug.Log($"Specific Scene Point {scenePoint} -> Projected Image Point: {projected}");
            }

            // --- Part 1.7: Reverse Projection from Image Points to Scene ---
            Debug.Log("==== Part 1.7: Reverse Projections ====");
            foreach (var imagePoint in specificImagePoints)
            {
                var projected = ProjectImageToScene(imagePoint, homography);
                Debug.Log($"Specific Image Point {imagePoint} -> Projected Scene Point: {projected}");
            }
        }
    }

    // Homography computation using non-linear optimization
    public static Matrix<double> ComputeHomography(List<Vector2> scenePoints, List<Vector2> imagePoints)
    {
        int n = scenePoints.Count;
        if (n < 4) throw new System.Exception("At least 4 points are required to compute the homography.");

        var A = Matrix<double>.Build.Dense(2 * n, 9);
        for (int i = 0; i < n; i++)
        {
            double x = scenePoints[i].x, y = scenePoints[i].y;
            double u = imagePoints[i].x, v = imagePoints[i].y;

            A.SetRow(2 * i, new[] { -x, -y, -1, 0, 0, 0, x * u, y * u, u });
            A.SetRow(2 * i + 1, new[] { 0, 0, 0, -x, -y, -1, x * v, y * v, v });
        }

        var svd = A.Svd();
        var h = svd.VT.Row(8).ToArray(); // Solution is the last row of V^T

        return Matrix<double>.Build.DenseOfArray(new double[,] {
            { h[0], h[1], h[2] },
            { h[3], h[4], h[5] },
            { h[6], h[7], h[8] }
        });
    }

    // Homography computation with correspondence similarity matrix
    public static Matrix<double> ComputeHomographyWithSimilarity(List<Vector2> scenePoints, List<Vector2> imagePoints, Matrix<double> similarity)
    {
        // Match the closest scene and image points based on similarity matrix
        var matchedScenePoints = new List<Vector2>();
        var matchedImagePoints = new List<Vector2>();

        for (int i = 0; i < similarity.RowCount; i++)
        {
            int bestMatch = 0;
            double bestValue = similarity[i, 0];
            for (int j = 1; j < similarity.ColumnCount; j++)
            {
                if (similarity[i, j] > bestValue)
                {
                    bestValue = similarity[i, j];
                    bestMatch = j;
                }
            }
            matchedScenePoints.Add(scenePoints[i]);
            matchedImagePoints.Add(imagePoints[bestMatch]);
        }

        return ComputeHomography(matchedScenePoints, matchedImagePoints);
    }

    // Project scene point onto the image
    public static Vector2 ProjectSceneToImage(Vector2 scenePoint, Matrix<double> homography)
    {
        var scene = Vector<double>.Build.DenseOfArray(new double[] { scenePoint.x, scenePoint.y, 1 });
        var projected = homography * scene;
        return new Vector2((float)(projected[0] / projected[2]), (float)(projected[1] / projected[2]));
    }

    // Project image point onto the scene
    public static Vector2 ProjectImageToScene(Vector2 imagePoint, Matrix<double> homography)
    {
        var invHomography = homography.Inverse();
        var image = Vector<double>.Build.DenseOfArray(new double[] { imagePoint.x, imagePoint.y, 1 });
        var projected = invHomography * image;
        return new Vector2((float)(projected[0] / projected[2]), (float)(projected[1] / projected[2]));
    }

    // Display results for 1.5 (manually matched points and errors)
    public static void DisplayResults()
    {
        // Example correspondences for demonstration
        List<Vector2> scenePoints = new List<Vector2> {
            new Vector2(0, 0), new Vector2(1, 0), new Vector2(1, 1), new Vector2(0, 1), new Vector2(0.5f, 0.5f)
        };
        List<Vector2> imagePoints = new List<Vector2> {
            new Vector2(100, 100), new Vector2(200, 100), new Vector2(200, 200), new Vector2(100, 200), new Vector2(150, 150)
        };

        var homography = ComputeHomography(scenePoints, imagePoints);
        Debug.Log("Computed Homography Matrix: \n" + homography.ToString());

        // Calculate and display projection errors
        for (int i = 0; i < scenePoints.Count; i++)
        {
            Vector2 projected = ProjectSceneToImage(scenePoints[i], homography);
            float error = Vector2.Distance(projected, imagePoints[i]);
            Debug.Log($"Point {i}: Error = {error}");
        }
    }
}
