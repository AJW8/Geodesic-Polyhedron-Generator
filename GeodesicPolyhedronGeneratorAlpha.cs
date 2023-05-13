using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This static class is used to generate geodesic polyhedral meshes using a division algorithm, with an isohedral shape as a starting point, and a number representing the order of complexity (the subdivision frequency - 1). Rather than evenly distributing the new vertices along each edge of the original shape and projecting them onto the unit sphere, this algorithm evenly distributes the new vertices along the arc of the unit sphere approximated by the edge. The result is a more even distribution of vertices compared to the standard subdivision method.
/// </summary>
public static class GeodesicPolyhedronGeneratorAlpha
{
    /// <summary>
    /// For each tetragonal (made of 2 triangles) face of a cube, augment and divide it into 2[(n + 1) ^ 2] smaller triangles (where n is the passed complexity) which are evenly distributed on the unit sphere with respect to the original face (where n is the passed complexity).
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <param name="t">A transform object is required for this algorithm. The original state of the transform is restored afterward.</param>
    /// <returns>A cube mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromCube(int complexity, Transform t)
    {
        Quaternion rotation = t.rotation; // get original rotation of transform so it can be restored later
        Vector3[] vertices = new Vector3[] { (new Vector3(-1, -1, -1)).normalized, (new Vector3(-1, -1, 1)).normalized, (new Vector3(-1, 1, -1)).normalized, (new Vector3(-1, 1, 1)).normalized, (new Vector3(1, -1, -1)).normalized, (new Vector3(1, -1, 1)).normalized, (new Vector3(1, 1, -1)).normalized, (new Vector3(1, 1, 1)).normalized }; // vertices for original shape
        int[] edges = new int[] { 0, 1, 0, 2, 0, 4, 1, 3, 1, 5, 2, 3, 2, 6, 3, 7, 4, 5, 4, 6, 5, 7, 6, 7 }; // edges for original shape (squares only)
        int[] squares = new int[] { 0, 1, 3, 2, 0, 4, 5, 1, 0, 2, 6, 4, 1, 5, 7, 3, 2, 3, 7, 6, 4, 6, 7, 5 }; // squares for original shape
        List<Vector3> newVertices = new List<Vector3>(vertices);
        List<int> newTriangles = new List<int>();
        for (int i = 0; i < edges.Length; i += 2) // for each edge of the original shape
        {
            // get both vertices that make up the current edge
            Vector3 v1 = vertices[edges[i]].normalized;
            Vector3 v2 = vertices[edges[i + 1]].normalized;
            Vector3 d = v1 - v2; // calculate the difference between both vertices
            float arcAngle = -Mathf.Sqrt(d.x * d.x + d.y * d.y + d.z * d.z) * 180f / Mathf.PI; // calculate the angle of the arc on the unit sphere approximated by the current edge
            for (int j = 0; j < complexity; j++) // for each new vertex to add
            {
                t.rotation = Quaternion.LookRotation(v1, v2); // rotate transform to look at the first vertex
                t.Rotate(arcAngle * (j + 1f) / (complexity + 1), 0, 0); // rotate transform along the axis given by the cross product of the two vertices to look at the new vertex
                newVertices.Add(t.forward); // add new vertex
            }
        }
        for (int i = 0; i < squares.Length; i += 4) // for each square of the original shape
        {
            // get the indices of the 4 vertices that make up the current square
            int v1 = squares[i];
            int v2 = squares[i + 1];
            int v3 = squares[i + 2];
            int v4 = squares[i + 3];
            // get the indices of the 3 edges that make up the current triangle
            int e1 = -1; // the index of the first new vertex on the edge between v1 and v2
            int e2 = -1; // the index of the first new vertex on the edge between v1 and v4
            int e3 = -1; // the index of the first new vertex on the edge between v2 and v3
            int e4 = -1; // the index of the first new vertex on the edge between v3 and v4
            for (int j = 0; j < edges.Length; j += 2) // for each edge of the original shape
            {
                if ((v1 == edges[j] || v1 == edges[j + 1]) && (v2 == edges[j] || v2 == edges[j + 1])) e1 = vertices.Length + j * complexity / 2;
                else if ((v1 == edges[j] || v1 == edges[j + 1]) && (v4 == edges[j] || v4 == edges[j + 1])) e2 = vertices.Length + j * complexity / 2;
                else if ((v2 == edges[j] || v2 == edges[j + 1]) && (v3 == edges[j] || v3 == edges[j + 1])) e3 = vertices.Length + j * complexity / 2;
                else if ((v3 == edges[j] || v3 == edges[j + 1]) && (v4 == edges[j] || v4 == edges[j + 1])) e4 = vertices.Length + j * complexity / 2;
            }
            // set up first row
            int[] currentRow = new int[complexity + 2];
            currentRow[0] = v1;
            for (int j = 0; j < complexity; j++) currentRow[j + 1] = e1 + j;
            currentRow[complexity + 1] = v2;
            for (int j = 0; j < complexity + 1; j++) // for each row
            {
                int[] newRow = new int[complexity + 2]; // this row will replace the current one at the end of the iteration
                if (j < complexity)
                {
                    newRow[0] = e2 + j;
                    for (int k = 0; k < complexity; k++) newRow[k + 1] = newVertices.Count + k;
                    newRow[complexity + 1] = e3 + (v2 < v3 ? j : complexity - 1 - j);
                }
                else // last row
                {
                    newRow[0] = v4;
                    for (int k = 0; k < complexity; k++) newRow[k + 1] = e4 + (v3 > v4 ? k : complexity - 1 - k);
                    newRow[complexity + 1] = v3;
                }
                for (int k = 0; k < complexity + 1; k++) // for each new pair of triangles to create in the current row
                {
                    if ((j + k) % 2 == 0) // first type of square split
                    {
                        // create first triangle
                        newTriangles.Add(currentRow[k]);
                        newTriangles.Add(currentRow[k + 1]);
                        newTriangles.Add(newRow[k + 1]);
                        // create second triangle
                        newTriangles.Add(currentRow[k]);
                        newTriangles.Add(newRow[k + 1]);
                        newTriangles.Add(newRow[k]);
                    }
                    else // second type of square split
                    {
                        // create first triangle
                        newTriangles.Add(currentRow[k]);
                        newTriangles.Add(currentRow[k + 1]);
                        newTriangles.Add(newRow[k]);
                        // create second triangle
                        newTriangles.Add(currentRow[k + 1]);
                        newTriangles.Add(newRow[k + 1]);
                        newTriangles.Add(newRow[k]);
                    }
                }
                if (j < complexity)
                {
                    // get both vertices that make up the current 'row'
                    Vector3 newRowStart = newVertices[newRow[0]].normalized;
                    Vector3 newRowEnd = newVertices[newRow[complexity + 1]].normalized;
                    Vector3 d = newRowStart - newRowEnd; // calculate the difference between both vertices
                    float arcAngle = -Mathf.Sqrt(d.x * d.x + d.y * d.y + d.z * d.z) * 180f / Mathf.PI; // calculate the angle of the arc on the unit sphere approximated by the current edge
                    for (int k = 0; k < complexity; k++) // for each new vertex to add
                    {
                        t.rotation = Quaternion.LookRotation(newRowStart, newRowEnd); // rotate transform to look at the first vertex
                        t.Rotate(arcAngle * (k + 1f) / (complexity + 1), 0, 0); // rotate transform along the axis given by the cross product of the two vertices to look at the new vertex
                        newVertices.Add(t.forward); // add new vertex
                    }
                }
                currentRow = newRow;
            }
        }
        t.rotation = rotation; // restore original transform rotation
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }

    /// <summary>
    /// For each face of a regular tetrahedron, divide it into (n + 1) ^ 2 smaller triangles which are evenly distributed on the unit sphere with respect to the original face (where n is the passed complexity).
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <param name="t">A transform object is required for this algorithm. The original state of the transform is restored afterward.</param>
    /// <returns>A tetrahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromTetrahedron(int complexity, Transform t)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { (new Vector3(-Mathf.Sqrt(2), 0, -1)).normalized, (new Vector3(Mathf.Sqrt(2), 0, -1)).normalized, (new Vector3(0, -Mathf.Sqrt(2), 1)).normalized, (new Vector3(0, Mathf.Sqrt(2), 1)).normalized }, new int[] { 0, 1, 0, 2, 0, 3, 1, 2, 1, 3, 2, 3 }, new int[] { 0, 1, 3, 0, 2, 1, 0, 3, 2, 1, 2, 3 }, complexity, t);
    }

    /// <summary>
    /// For each triangular face of an isohedral shape with the passed data, divide it into (n + 1) ^ 2 smaller triangles which are evenly distributed on the unit sphere with respect to the original face (where n is the passed complexity).
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <param name="t">A transform object is required for this algorithm. The original state of the transform is restored afterward.</param>
    /// <returns>An octahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromOctahedron(int complexity, Transform t)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, -1, 0), new Vector3(0, 1, 0), new Vector3(0, 0, -1), new Vector3(0, 0, 1) }, new int[] { 0, 2, 0, 3, 0, 4, 0, 5, 1, 2, 1, 3, 1, 4, 1, 5, 2, 4, 2, 5, 3, 4, 3, 5 }, new int[] { 0, 2, 5, 0, 4, 2, 0, 3, 4, 0, 5, 3, 1, 2, 4, 1, 5, 2, 1, 3, 5, 1, 4, 3 }, complexity, t);
    }

    /// <summary>
    /// For each triangular face of an isohedral shape with the passed data, divide it into (n + 1) ^ 2 smaller triangles which are evenly distributed on the unit sphere with respect to the original face (where n is the passed complexity).
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <param name="t">A transform object is required for this algorithm. The original state of the transform is restored afterward.</param>
    /// <returns>A icosahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromIcosahedron(int complexity, Transform t)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { (new Vector3(-1, -(1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(-1, (1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(1, -(1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(1, (1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(0, -1, -(1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, -1, (1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, 1, -(1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, 1, (1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(-(1 + Mathf.Sqrt(5)) / 2, 0, -1)).normalized, (new Vector3((1 + Mathf.Sqrt(5)) / 2, 0, -1)).normalized, (new Vector3(-(1 + Mathf.Sqrt(5)) / 2, 0, 1)).normalized, (new Vector3((1 + Mathf.Sqrt(5)) / 2, 0, 1)).normalized }, new int[] { 0, 2, 0, 4, 0, 5, 0, 8, 0, 10, 1, 3, 1, 6, 1, 7, 1, 8, 1, 10, 2, 4, 2, 5, 2, 9, 2, 11, 3, 6, 3, 7, 3, 9, 3, 11, 4, 6, 4, 8, 4, 9, 5, 7, 5, 10, 5, 11, 6, 8, 6, 9, 7, 10, 7, 11, 8, 10, 9, 11 }, new int[] { 0, 2, 5, 0, 4, 2, 0, 5, 10, 0, 8, 4, 0, 10, 8, 1, 6, 8, 1, 10, 7, 1, 8, 10, 2, 4, 9, 2, 9, 11, 2, 11, 5, 4, 8, 6, 4, 6, 9, 5, 7, 10, 5, 11, 7, 1, 3, 6, 1, 7, 3, 3, 9, 6, 3, 7, 11, 3, 11, 9 }, complexity, t);
    }

    /// <summary>
    /// For each triangular face of an isohedral shape with the passed data, divide it into (n + 1) ^ 2 smaller triangles which are evenly distributed on the unit sphere with respect to the original face (where n is the passed complexity).
    /// </summary>
    /// <param name="vertices"></param>
    /// <param name="edges">The vertices of the original shape.</param>
    /// <param name="triangles">The triangular faces of the original shape.</param>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <param name="t">A transform object is required for this algorithm. The original state of the transform is restored afterward.</param>
    /// <returns>A copy of the passed mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    private static Mesh GenerateGeodesicPolyhedron(Vector3[] vertices, int[] edges, int[] triangles, int complexity, Transform t)
    {
        Quaternion rotation = t.rotation; // get original rotation of transform so it can be restored later
        List<Vector3> newVertices = new List<Vector3>(vertices);
        List<int> newTriangles = new List<int>();
        for (int i = 0; i < edges.Length; i += 2) // for each edge of the original shape
        {
            // get both vertices that make up the current edge
            Vector3 v1 = vertices[edges[i]].normalized;
            Vector3 v2 = vertices[edges[i + 1]].normalized;
            Vector3 d = v1 - v2; // calculate the difference between both vertices
            float arcAngle = -Mathf.Sqrt(d.x * d.x + d.y * d.y + d.z * d.z) * 180f / Mathf.PI; // calculate the angle of the arc on the unit sphere approximated by the current edge
            for (int j = 0; j < complexity; j++) // for each new vertex to add
            {
                t.rotation = Quaternion.LookRotation(v1, v2); // rotate transform to look at the first vertex
                t.Rotate(arcAngle * (j + 1f) / (complexity + 1), 0, 0); // rotate transform along the axis given by the cross product of the two vertices to look at the new vertex
                newVertices.Add(t.forward); // add new vertex
            }
        }
        for (int i = 0; i < triangles.Length; i += 3) // for each triangle of the original shape
        {
            // get the indices of the 3 vertices that make up the current triangle
            int v1 = triangles[i];
            int v2 = triangles[i + 1];
            int v3 = triangles[i + 2];
            // get the indices of the 3 edges that make up the current triangle
            int e1 = -1; // the index of the first new vertex on the edge between v1 and v2
            int e2 = -1; // the index of the first new vertex on the edge between v1 and v3
            int e3 = -1; // the index of the first new vertex on the edge between v2 and v3
            for (int j = 0; j < edges.Length; j += 2) // for each edge of the original shape
            {
                if ((v1 == edges[j] || v1 == edges[j + 1]) && (v2 == edges[j] || v2 == edges[j + 1])) e1 = vertices.Length + j * complexity / 2;
                else if ((v1 == edges[j] || v1 == edges[j + 1]) && (v3 == edges[j] || v3 == edges[j + 1])) e2 = vertices.Length + j * complexity / 2;
                else if ((v2 == edges[j] || v2 == edges[j + 1]) && (v3 == edges[j] || v3 == edges[j + 1])) e3 = vertices.Length + j * complexity / 2;
            }
            // set up first 'row'
            int[] currentRow = new int[complexity + 2];
            currentRow[0] = v1;
            for (int j = 0; j < complexity; j++) currentRow[j + 1] = e1 + j;
            currentRow[complexity + 1] = v2;
            for (int j = 0; j < complexity; j++) // for each 'row'
            {
                int[] newRow = new int[complexity + 1 - j]; // this row will replace the current one at the end of the iteration
                newRow[0] = e2 + j;
                for (int k = 0; k < complexity - 1 - j; k++) newRow[k + 1] = newVertices.Count + k;
                newRow[complexity - j] = e3 + (v2 < v3 ? j : complexity - 1 - j);
                for (int k = 0; k < complexity - j; k++) // for each new pair of triangles (1 flat top, 1 pointed top) to create in the current 'row'
                {
                    // create current flat top triangle
                    newTriangles.Add(currentRow[k]);
                    newTriangles.Add(currentRow[k + 1]);
                    newTriangles.Add(newRow[k]);
                    // create current pointed top triangle
                    newTriangles.Add(currentRow[k + 1]);
                    newTriangles.Add(newRow[k + 1]);
                    newTriangles.Add(newRow[k]);
                }
                // get both vertices that make up the current 'row'
                Vector3 newRowStart = newVertices[newRow[0]].normalized;
                Vector3 newRowEnd = newVertices[newRow[complexity - j]].normalized;
                Vector3 d = newRowStart - newRowEnd; // calculate the difference between both vertices
                float arcAngle = -Mathf.Sqrt(d.x * d.x + d.y * d.y + d.z * d.z) * 180f / Mathf.PI; // calculate the angle of the arc on the unit sphere approximated by the current edge
                for (int k = 0; k < complexity; k++) // for each new vertex to add
                {
                    t.rotation = Quaternion.LookRotation(newRowStart, newRowEnd); // rotate transform to look at the first vertex
                    t.Rotate(arcAngle * (k + 1f) / (complexity + 1), 0, 0); // rotate transform along the axis given by the cross product of the two vertices to look at the new vertex
                    newVertices.Add(t.forward); // add new vertex
                }
                // create final triangle in 'row'
                newTriangles.Add(currentRow[complexity - j]);
                newTriangles.Add(currentRow[complexity + 1 - j]);
                newTriangles.Add(newRow[complexity - j]);
                currentRow = newRow;
            }
            // create final triangle
            newTriangles.Add(currentRow[0]);
            newTriangles.Add(currentRow[1]);
            newTriangles.Add(v3);
        }
        t.rotation = rotation; // restore original transform rotation
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }
}
