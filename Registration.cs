using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Registration : MonoBehaviour
{
    public static SVDResult SVDStep(Vector3[] sourcePoints, Vector3[] targetPoints)
    {
        // Check if lengths match
        if (sourcePoints.Length != targetPoints.Length)
        {
            throw new Exception("Could not perform Horn step: array lengths did not match.");
        }

        // Variables
        int m = 3; // dimension
        int n = sourcePoints.Length; // number of point pairs

        // STEP 1
        // Find centroids (mean) of source and target points
        Vector3 sourceCentroid = Vector3.zero;
        Vector3 targetCentroid = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            // CODE HERE
            sourceCentroid.x += sourcePoints[i].x;
            sourceCentroid.y += sourcePoints[i].y;
            sourceCentroid.z += sourcePoints[i].z;
            targetCentroid.x += targetPoints[i].x;
            targetCentroid.y += targetPoints[i].y;
            targetCentroid.z += targetPoints[i].z;
        }
        // CODE HERE
        sourceCentroid = new Vector3(sourceCentroid.x / n, sourceCentroid.y / n, sourceCentroid.z / n);
        targetCentroid = new Vector3(targetCentroid.x / n, targetCentroid.y / n, targetCentroid.z / n);

        // STEP 2
        // Construct matrices from source and target points (alglib operates with double arrays)
        var sourceMatrix = new double[n, m]; // Matrix P'
        var targetMatrix = new double[m, n]; // Matrix Q'
        for (int i = 0; i < n; ++i)
        {
            // CODE HERE
            sourceMatrix[i, 0] = sourcePoints[i].x - sourceCentroid.x;
            sourceMatrix[i, 1] = sourcePoints[i].y - sourceCentroid.y;
            sourceMatrix[i, 2] = sourcePoints[i].z - sourceCentroid.z;
            targetMatrix[0, i] = targetPoints[i].x - targetCentroid.x;
            targetMatrix[1, i] = targetPoints[i].y - targetCentroid.y;
            targetMatrix[2, i] = targetPoints[i].z - targetCentroid.z;
        }

        // STEP 3
        // Compute cross-covariance matrix using alglib's rmatrixgemm function: https://www.alglib.net/translator/man/manual.csharp.html#sub_rmatrixgemm
        var resultMatrix = new double[m, m];
        // Compute W = Q*P^T
        // UNCOMMENT AND COMPLETE CODE BELOW
        alglib.rmatrixgemm(m, m, n, 1, targetMatrix, 0, 0, 0, sourceMatrix, 0, 0, 0, 0, ref resultMatrix, 0, 0);
        // M, N, K
        // A is a MxK matrix: m x n
        // B is a KxN matrix: n x m

        // STEP 4
        // Decompose W=UDV^T using using alglib's rmatrixsvd function: https://www.alglib.net/translator/man/manual.csharp.html#sub_rmatrixsvd
        // The algorithm calculates the singular value decomposition of a matrix of size MxN: A = U * S * V^T

        var W = new double[m];
        var U = new double[m, m];
        var VT = new double[m, m];
        // UNCOMMENT AND COMPLETE CODE BELOW
        alglib.rmatrixsvd(resultMatrix, m, m, 2, 2, 2, out W, out U, out VT);
        // double[,] a, int m, int n, int uneeded, int vtneeded, int additionalmemory, out double [] w, out double [,] u, out double [,] vt)
        
        // STEP 5
        // Consider the special case of reflection discussed in the lecture, i.e. check whether det(U)*det(V^T) < 0. If so, modify U or V^T accordingly.
        if (alglib.rmatrixdet(U) * alglib.rmatrixdet(VT) < 0)
        {
            // CODE HERE
            var determinant = alglib.rmatrixdet(U) * alglib.rmatrixdet(VT);
            var diagonal = new double[m, m];

            // CODE HERE
            diagonal[0, 0] = 1f;
            diagonal[0, 1] = 0f;
            diagonal[0, 2] = 0f;
            diagonal[1, 0] = 0f;
            diagonal[1, 1] = 1f;
            diagonal[1, 2] = 0f;
            diagonal[2, 0] = 0f;
            diagonal[2, 1] = 0f;
            diagonal[2, 2] = determinant;

            alglib.rmatrixgemm(m, m, m, 1, U, 0, 0, 0, diagonal, 0, 0, 1, 0, ref U, 0, 0);
        }

        // STEP 6
        // Compute rotation R=U*V^T
        var R = new double[m, m];
        // UNCOMMENT AND COMPLETE CODE BELOW
        alglib.rmatrixgemm(m, m, m, 1, U, 0, 0, 0, VT, 0, 0, 0, 0, ref R, 0, 0);
        
        // Compute translation T
        var T = targetCentroid - sourceCentroid;

        // Return result as a new SVDResult (Mat2Quat converts R to a Quaternion representation)
        return new SVDResult(sourceCentroid, targetCentroid, Mat2Quat(R), T);
    }

    public static SVDResult ICP(List<Vector3> sampledPoints, List<Vector3> meshVertices, int maxIterations, double epsilon)
    {
        // Convert source and target points to 2D arrays (double[, ])
        var source = To2dArray(sampledPoints);
        var target = To2dArray(meshVertices);

        // Build k-d tree for target 3D model points
        var tags = Enumerable.Range(0, target.GetLength(0)).ToArray();
        alglib.kdtree kdt;
        alglib.kdtreebuildtagged(target, tags, target.GetLength(1), 0, 2, out kdt);

        // Copy source points (copied points will be altered during ICP)
        var sourceCopy = source;

        // Assign target points array, will contain nearest neighbor points on target model
        Vector3[] targetPoints = null;

        // STEP 1
        // Find indices of nearest neighbor for each sampled point and compute starting RMSE
        // Complete the implementation of the method FindNearestNeighbors(...)
        double rmse;
        var nearestNeighborsIdx = FindNearestNeighbors(kdt, sourceCopy, out rmse);
        Debug.Log("Idx NN: " + nearestNeighborsIdx);

        // ICP
        for (int i = 0; i < maxIterations; ++i)
		{
            // Select 3D points from indices (result of the nearest neighbors method)
            // If no indices are given, this is just a conversion from a 2D double array to an array of Vector3, which is our input format for the SVD step
            var sourcePoints = SelectPointsByIndices(sourceCopy);
            targetPoints = SelectPointsByIndices(target, nearestNeighborsIdx);

            // STEP 2
            // Perform SVD step
            // UNCOMMENT AND COMPLETE CODE BELOW
            var svdResult = SVDStep(sourcePoints, targetPoints);
            
            // Transform points according to SVD result
            // ONLY UNCOMMENT, NO NEED FOR OTHER CHANGES
            sourceCopy = TransformPoints(svdResult, sourceCopy);

            // STEP 3
            // Again, find indices of nearest neighbor for each sampled point and compute RMSE
            double newRmse;
            // CODE HERE
            var NearestNeighborsIdx = FindNearestNeighbors(kdt, sourceCopy, out newRmse);

            // STEP 4
            // Log iteration number and current RMSE
            // UNCOMMENT AND COMPLETE CODE BELOW
            Debug.Log("Iteration number: " + i + "\nCurrent RMSE: " + newRmse);
            // I logged the raw iteration number, alternatively output i+1

            // STEP 5
            // Check RMSE termination criteria
            // CODE HERE
            if (Math.Abs(rmse - newRmse) < epsilon){ // took abs because sometimes the newRmse is higher than the old
                break;
            }

            // Update RMSE
            // ONLY UNCOMMENT, NO NEED FOR OTHER CHANGES
            rmse = newRmse;
        }

        // After termination, the best, i.e. final, correspondences are kown.
        // However, we neither kept track of the transformations (SVDSteps), nor transform our game object after every step
        // Instead, we rely on the final, best correspondences and compute a single transformation based on the initial source and the last, i.e. best, target point correspondences
        // This way, we can simply transform the game object of the preoperative 3D model once (in the SceneManager.cs script)
        var originalSourcePoints = SelectPointsByIndices(source);
        return SVDStep(originalSourcePoints, targetPoints);
    }

    private static int[] FindNearestNeighbors(alglib.kdtree kdt, double[,] queryPoints, out double rmse)
    {
        var output = new int[queryPoints.GetLength(0)]; // output array
        double squaredDistSum = 0; // sum of squared distances between query points and their nearest neighbors

        // Iterate over all query points
        for (int i = 0; i < queryPoints.GetLength(0); ++i)
        {
            // Run nearest neighbor query using alglib's kdtreequeryknn function: https://www.alglib.net/translator/man/manual.csharp.html#sub_kdtreequeryknn
            var queryPoint = new double[3] { queryPoints[i, 0], queryPoints[i, 1], queryPoints[i, 2] };
            // UNCOMMENT AND COMPLETE CODE BELOW
            alglib.kdtreequeryknn(kdt, queryPoint, 1);

            // Get result of last query using alglib's kdtreequeryresultstags function: https://www.alglib.net/translator/man/manual.csharp.html#sub_kdtreequeryresultstags
            int[] idx = new int[1];
            // UNCOMMENT AND COMPLETE CODE BELOW
            alglib.kdtreequeryresultstags(kdt, ref idx);
            output[i] = idx[0];

            // Get distance of last query using alglib's kdtreequeryresultsdistances function: https://www.alglib.net/translator/man/manual.csharp.html#sub_kdtreequeryresultsdistances
            double[] d = new double[1];
            // UNCOMMENT AND COMPLETE CODE BELOW
            alglib.kdtreequeryresultsdistances(kdt, ref d);
            squaredDistSum += Math.Pow(d[0], 2);
        }

        // Compute RMSE
        rmse = Math.Sqrt(squaredDistSum / queryPoints.GetLength(0));

        return output;
    }

    #region Helper Methods

    private static double[,] To2dArray(List<Vector3> input)
    {
        var output = new double[input.Count, 3];

        int i = 0;
        foreach (var element in input)
        {
            output[i, 0] = element.x;
            output[i, 1] = element.y;
            output[i, 2] = element.z;
            ++i;
        }

        return output;
    }

    private static double[,] To2dArray(Vector3[] input)
    {
        var output = new double[input.Length, 3];

        for (int i = 0; i < input.Length; ++i)
        {
            output[i, 0] = input[i].x;
            output[i, 1] = input[i].y;
            output[i, 2] = input[i].z;
        }

        return output;
    }

    private static Vector3[] SelectPointsByIndices(double[,] input, int[] indices = null)
    {
        Vector3[] output;
        if (indices == null)
        {
            output = new Vector3[input.GetLength(0)];
            for (int i = 0; i < input.GetLength(0); ++i)
            {
                output[i].x = (float)input[i, 0];
                output[i].y = (float)input[i, 1];
                output[i].z = (float)input[i, 2];
            }
        }
        else
        {
            output = new Vector3[indices.Length];
            for (int i = 0; i < indices.Length; ++i)
            {
                output[i].x = (float)input[indices[i], 0];
                output[i].y = (float)input[indices[i], 1];
                output[i].z = (float)input[indices[i], 2];
            }
        }
        return output;
    }

    private static double[,] TransformPoints(SVDResult hornResult, double[,] input)
    {
        var output = new double[input.GetLength(0), input.GetLength(1)];

        for (int i = 0; i < input.GetLength(0); ++i)
        {
            var vec = new Vector3((float)input[i, 0], (float)input[i, 1], (float)input[i, 2]);

            vec -= hornResult.SourceCentroid;   // Translate back
            vec = hornResult.Rotation * vec;    // Rotate
            vec += hornResult.SourceCentroid;   // Translate forth
            vec += hornResult.Translation;      // Translate

            output[i, 0] = vec.x;
            output[i, 1] = vec.y;
            output[i, 2] = vec.z;
        }

        return output;
    }

    public static Quaternion Mat2Quat(double[,] R)
    {
        Quaternion q = new Quaternion();
        q.w = (float)Math.Sqrt(Math.Max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2;
        q.x = (float)Math.Sqrt(Math.Max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2;
        q.y = (float)Math.Sqrt(Math.Max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2;
        q.z = (float)Math.Sqrt(Math.Max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2;
        q.x *= Math.Sign(q.x * (R[2, 1] - R[1, 2]));
        q.y *= Math.Sign(q.y * (R[0, 2] - R[2, 0]));
        q.z *= Math.Sign(q.z * (R[1, 0] - R[0, 1]));
        return q;
    }

    #endregion

}

public class SVDResult
{
    public Vector3 SourceCentroid;
    public Vector3 TargetCentroid;
    public Quaternion Rotation;
    public Vector3 Translation;

    public SVDResult(Vector3 sourceCentroid, Vector3 targetCentroid, Quaternion rotation, Vector3 translation)
    {
        SourceCentroid = sourceCentroid;
        TargetCentroid = targetCentroid;
        Rotation = rotation;
        Translation = translation;
    }
}