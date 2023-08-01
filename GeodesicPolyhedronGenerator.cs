using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This static class is used to generate geodesic polyhedral meshes using a division algorithm, with an isohedral shape as a starting point, and a number representing the order of complexity (the subdivision frequency - 1). The algorithm allows for a better approximation of a unit sphere the higher the complexity, and avoids producing redundant vertices/triangles.
/// </summary>
public static class GeodesicPolyhedronGenerator
{
    /// <summary>
    /// For each tetragonal (made of 2 triangles) face of a cube, augment and divide it into 2[(n + 1) ^ 2] smaller triangles (where n is the passed complexity), and project all new vertices onto the unit sphere.
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <returns>A cube mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromCube(int complexity)
    {
        Vector3[] vertices = new Vector3[] { (new Vector3(-1, -1, -1)).normalized, (new Vector3(-1, -1, 1)).normalized, (new Vector3(-1, 1, -1)).normalized, (new Vector3(-1, 1, 1)).normalized, (new Vector3(1, -1, -1)).normalized, (new Vector3(1, -1, 1)).normalized, (new Vector3(1, 1, -1)).normalized, (new Vector3(1, 1, 1)).normalized }; // vertices for original shape
        int[] edges = new int[] { 0, 1, 0, 2, 0, 4, 1, 3, 1, 5, 2, 3, 2, 6, 3, 7, 4, 5, 4, 6, 5, 7, 6, 7 }; // edges for original shape (squares only)
        int[] squares = new int[] { 0, 1, 3, 2, 0, 4, 5, 1, 0, 2, 6, 4, 1, 5, 7, 3, 2, 3, 7, 6, 4, 6, 7, 5 }; // squares for original shape
        List<Vector3> newVertices = new List<Vector3>(vertices);
        List<int> newTriangles = new List<int>();
        for (int i = 0; i < edges.Length; i += 2) // for each edge of the original shape
        {
            // get both vertices that make up the current edge
            Vector3 v1 = vertices[edges[i]];
            Vector3 v2 = vertices[edges[i + 1]];
            for (int j = 0; j < complexity; j++) newVertices.Add((v1 + (v2 - v1) * (j + 1f) / (complexity + 1)).normalized); // add new vertices to the current edge and project each one onto the unit sphere
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
                    Vector3 newRowStart = newVertices[newRow[0]];
                    Vector3 newRowEnd = newVertices[newRow[complexity + 1]];
                    for (int k = 0; k < complexity; k++) newVertices.Add((newRowStart + (newRowEnd - newRowStart) * (k + 1f) / (complexity + 1)).normalized); // create new vertex
                }
                currentRow = newRow;
            }
        }
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }

    /// <summary>
    /// For each face of a regular tetrahedron, divide it into (n + 1) ^ 2 smaller triangles (where n is the passed complexity), and project all new vertices onto the unit sphere.
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <returns>A tetrahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromTetrahedron(int complexity)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { (new Vector3(-Mathf.Sqrt(2), 0, -1)).normalized, (new Vector3(Mathf.Sqrt(2), 0, -1)).normalized, (new Vector3(0, -Mathf.Sqrt(2), 1)).normalized, (new Vector3(0, Mathf.Sqrt(2), 1)).normalized }, new int[] { 0, 1, 0, 2, 0, 3, 1, 2, 1, 3, 2, 3 }, new int[] { 0, 1, 3, 0, 2, 1, 0, 3, 2, 1, 2, 3 }, complexity);
    }

    /// <summary>
    /// For each face of a regular octahedron, divide it into (n + 1) ^ 2 smaller triangles (where n is the passed complexity), and project all new vertices onto the unit sphere.
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <returns>An octahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromOctahedron(int complexity)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, -1, 0), new Vector3(0, 1, 0), new Vector3(0, 0, -1), new Vector3(0, 0, 1) }, new int[] { 0, 2, 0, 3, 0, 4, 0, 5, 1, 2, 1, 3, 1, 4, 1, 5, 2, 4, 2, 5, 3, 4, 3, 5 }, new int[] { 0, 2, 5, 0, 4, 2, 0, 3, 4, 0, 5, 3, 1, 2, 4, 1, 5, 2, 1, 3, 5, 1, 4, 3 }, complexity);
    }

    /// <summary>
    /// For each face of a regular icosahedron, divide it into (n + 1) ^ 2 smaller triangles (where n is the passed complexity), and project all new vertices onto the unit sphere.
    /// </summary>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <returns>A icosahedral mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    public static Mesh GenerateGeodesicPolyhedronFromIcosahedron(int complexity)
    {
        return GenerateGeodesicPolyhedron(new Vector3[] { (new Vector3(-1, -(1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(-1, (1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(1, -(1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(1, (1 + Mathf.Sqrt(5)) / 2, 0)).normalized, (new Vector3(0, -1, -(1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, -1, (1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, 1, -(1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(0, 1, (1 + Mathf.Sqrt(5)) / 2)).normalized, (new Vector3(-(1 + Mathf.Sqrt(5)) / 2, 0, -1)).normalized, (new Vector3((1 + Mathf.Sqrt(5)) / 2, 0, -1)).normalized, (new Vector3(-(1 + Mathf.Sqrt(5)) / 2, 0, 1)).normalized, (new Vector3((1 + Mathf.Sqrt(5)) / 2, 0, 1)).normalized }, new int[] { 0, 2, 0, 4, 0, 5, 0, 8, 0, 10, 1, 3, 1, 6, 1, 7, 1, 8, 1, 10, 2, 4, 2, 5, 2, 9, 2, 11, 3, 6, 3, 7, 3, 9, 3, 11, 4, 6, 4, 8, 4, 9, 5, 7, 5, 10, 5, 11, 6, 8, 6, 9, 7, 10, 7, 11, 8, 10, 9, 11 }, new int[] { 0, 2, 5, 0, 4, 2, 0, 5, 10, 0, 8, 4, 0, 10, 8, 1, 6, 8, 1, 10, 7, 1, 8, 10, 2, 4, 9, 2, 9, 11, 2, 11, 5, 4, 8, 6, 4, 6, 9, 5, 7, 10, 5, 11, 7, 1, 3, 6, 1, 7, 3, 3, 9, 6, 3, 7, 11, 3, 11, 9 }, complexity);
    }

    /// <summary>
    /// For each triangular face of an isohedral shape with the passed data, divide it into (n + 1) ^ 2 smaller triangles (where n is the passed complexity), and project all new vertices onto the unit sphere.
    /// </summary>
    /// <param name="vertices">The vertices of the original shape.</param>
    /// <param name="edges">The edges of the original shape.</param>
    /// <param name="triangles">The triangular faces of the original shape.</param>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape (note: passing 0 to this parameter will return the original mesh unchanged).</param>
    /// <returns>A copy of the passed mesh with the subdivision algorithm applied to it to the specified order of complexity.</returns>
    private static Mesh GenerateGeodesicPolyhedron(Vector3[] vertices, int[] edges, int[] triangles, int complexity)
    {
        List<Vector3> newVertices = new List<Vector3>(vertices);
        List<int> newTriangles = new List<int>();
        for (int i = 0; i < edges.Length; i += 2) // for each edge of the original shape
        {
            // get both vertices that make up the current edge
            Vector3 v1 = vertices[edges[i]];
            Vector3 v2 = vertices[edges[i + 1]];
            for (int j = 0; j < complexity; j++) newVertices.Add((v1 + (v2 - v1) * (j + 1f) / (complexity + 1)).normalized); // add new vertices to the current edge and project each one onto the unit sphere
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
                Vector3 newRowStart = newVertices[newRow[0]];
                Vector3 newRowEnd = newVertices[newRow[complexity - j]];
                for (int k = 0; k < complexity - 1 - j; k++) newVertices.Add((newRowStart + (newRowEnd - newRowStart) * (k + 1f) / (complexity - j)).normalized); // create new vertex
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
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }

    /// <summary>
    /// Produce the dual of a tetrahedron whose faces have been subdivided according to the specified level of complexity.
    /// </summary>
    /// <param name="class1">True if making a polyhedron of Class 1 (m, 0), false if making one of Class 2 (m, m).</param>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape.</param>
    /// <returns>The dual of a geodesic tetrahedron of Class 1 or 2 built according to the specified level of complexity.</returns>
    public static Mesh GenerateDualGeodesicTetrahedron(bool class1, int complexity)
    {
        Mesh mesh = GenerateGeodesicPolyhedronFromTetrahedron(complexity);
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        List<Vector3> newVertices = new List<Vector3>();
        List<int> newTriangles = new List<int>();
        List<int> hexagons = new List<int>();
        if (class1) // class 1 polyhedron (m, 0)
        {
            for (int i = 0; i < triangles.Length; i += 3) newVertices.Add((vertices[triangles[i]] + vertices[triangles[i + 1]] + vertices[triangles[i + 2]]).normalized); // add new vertex in the centre of each face of the dual shape
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    if (adjacentFaces.Count == 3) newTriangles.Add(adjacentFaces[currentIndex]);
                    else if (adjacentFaces.Count == 6) hexagons.Add(adjacentFaces[currentIndex]);
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j]) // add point to current face
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
        }
        else // class 2 polyhedron (m, m)
        {
            List<int> edges = new List<int>();
            List<int> newEdges = new List<int>();
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    edges.Add(i);
                    edges.Add(nextPoints[currentIndex]);
                    newEdges.Add(newVertices.Count);
                    if (adjacentFaces.Count == 3) newTriangles.Add(newVertices.Count);
                    else if (adjacentFaces.Count == 6) hexagons.Add(newVertices.Count);
                    newVertices.Add((vertices[i] + (vertices[nextPoints[currentIndex]] - vertices[i]) / 3).normalized); // add new vertex a third of the way along the edge between the two vertices and project it onto the unit sphere
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j])
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
            for (int i = 0; i < triangles.Length; i++) // for each triangle of the dual shape
            {
                List<int> adjacentVertices = new List<int>();
                List<int> newAdjacentVertices = new List<int>();
                for (int j = 0; j < edges.Count; j += 2)
                {
                    if (triangles[i] == edges[j])
                    {
                        adjacentVertices.Add(edges[j + 1]);
                        newAdjacentVertices.Add(newEdges[j / 2]);
                    }
                }
                for (int j = 0; j < adjacentVertices.Count; j++)
                {
                    if (triangles[i % 3 < 2 ? i + 1 : i - 2] == adjacentVertices[j] && triangles[i % 3 == 0 ? i + 2 : i - 1] == adjacentVertices[(j + 1) % adjacentVertices.Count]) // add these two vertices to current hexagon
                    {
                        hexagons.Add(newAdjacentVertices[(j + 1) % newAdjacentVertices.Count]);
                        hexagons.Add(newAdjacentVertices[j]);
                    }
                }
            }
        }
        Vector3 newVertex = new Vector3(0, 0, 0);
        for (int i = 0; i < hexagons.Count; i++) // place new vertex in the centre of each hexagonal face and connect it to the edges with triangles
        {
            newVertex += newVertices[hexagons[i]] / 6;
            newTriangles.Add(hexagons[i]);
            if (i % 6 < 5)
            {
                newTriangles.Add(hexagons[i + 1]);
                newTriangles.Add(newVertices.Count);
            }
            else
            {
                newTriangles.Add(hexagons[i - 5]);
                newTriangles.Add(newVertices.Count);
                newVertices.Add(newVertex);
                newVertex = new Vector3(0, 0, 0);
            }
        }
        int[] newTriangleArray = newTriangles.ToArray();
        for (int i = 0; i < newTriangleArray.Length; i += 3)
        {
            int t = newTriangleArray[i + 1];
            newTriangleArray[i + 1] = newTriangleArray[i + 2];
            newTriangleArray[i + 2] = t;
        }
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangleArray
        };
    }

    /// <summary>
    /// Produce the dual of an octahedron whose faces have been subdivided according to the specified level of complexity.
    /// </summary>
    /// <param name="class1">True if making a polyhedron of Class 1 (m, 0), false if making one of Class 2 (m, m).</param>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape.</param>
    /// <returns>The dual of a geodesic octahedron of Class 1 or 2 built according to the specified level of complexity.</returns>
    public static Mesh GenerateDualGeodesicOctahedron(bool class1, int complexity)
    {
        Mesh mesh = GenerateGeodesicPolyhedronFromOctahedron(complexity);
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        List<Vector3> newVertices = new List<Vector3>();
        List<int> newTriangles = new List<int>();
        List<int> squares = new List<int>();
        List<int> hexagons = new List<int>();
        if (class1) // class 1 polyhedron (m, 0)
        {
            for (int i = 0; i < triangles.Length; i += 3) newVertices.Add((vertices[triangles[i]] + vertices[triangles[i + 1]] + vertices[triangles[i + 2]]).normalized); // add new vertex in the centre of each face of the dual shape
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    if (adjacentFaces.Count == 4) squares.Add(adjacentFaces[currentIndex]);
                    else if (adjacentFaces.Count == 6) hexagons.Add(adjacentFaces[currentIndex]);
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j]) // add point to current face
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
        }
        else // class 2 polyhedron (m, m)
        {
            List<int> edges = new List<int>();
            List<int> newEdges = new List<int>();
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    edges.Add(i);
                    edges.Add(nextPoints[currentIndex]);
                    newEdges.Add(newVertices.Count);
                    if (adjacentFaces.Count == 4) squares.Add(newVertices.Count);
                    else if (adjacentFaces.Count == 6) hexagons.Add(newVertices.Count);
                    newVertices.Add((vertices[i] + (vertices[nextPoints[currentIndex]] - vertices[i]) / 3).normalized); // add new vertex a third of the way along the edge between the two vertices and project it onto the unit sphere
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j])
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
            for (int i = 0; i < triangles.Length; i++) // for each triangle of the dual shape
            {
                List<int> adjacentVertices = new List<int>();
                List<int> newAdjacentVertices = new List<int>();
                for (int j = 0; j < edges.Count; j += 2)
                {
                    if (triangles[i] == edges[j])
                    {
                        adjacentVertices.Add(edges[j + 1]);
                        newAdjacentVertices.Add(newEdges[j / 2]);
                    }
                }
                for (int j = 0; j < adjacentVertices.Count; j++)
                {
                    if (triangles[i % 3 < 2 ? i + 1 : i - 2] == adjacentVertices[j] && triangles[i % 3 == 0 ? i + 2 : i - 1] == adjacentVertices[(j + 1) % adjacentVertices.Count]) // add these two vertices to current hexagon
                    {
                        hexagons.Add(newAdjacentVertices[(j + 1) % newAdjacentVertices.Count]);
                        hexagons.Add(newAdjacentVertices[j]);
                    }
                }
            }
        }
        Vector3 newVertex = new Vector3(0, 0, 0);
        for (int i = 0; i < squares.Count; i++) // place new vertex in the centre of each square face and connect it to the edges with triangles
        {
            newVertex += newVertices[squares[i]] / 4;
            newTriangles.Add(squares[i]);
            if (i % 4 < 3)
            {
                newTriangles.Add(squares[i + 1]);
                newTriangles.Add(newVertices.Count);
            }
            else
            {
                newTriangles.Add(squares[i - 3]);
                newTriangles.Add(newVertices.Count);
                newVertices.Add(newVertex);
                newVertex = new Vector3(0, 0, 0);
            }
        }
        for (int i = 0; i < hexagons.Count; i++) // place new vertex in the centre of each hexagonal face and connect it to the edges with triangles
        {
            newVertex += newVertices[hexagons[i]] / 6;
            newTriangles.Add(hexagons[i]);
            if (i % 6 < 5)
            {
                newTriangles.Add(hexagons[i + 1]);
                newTriangles.Add(newVertices.Count);
            }
            else
            {
                newTriangles.Add(hexagons[i - 5]);
                newTriangles.Add(newVertices.Count);
                newVertices.Add(newVertex);
                newVertex = new Vector3(0, 0, 0);
            }
        }
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }

    /// <summary>
    /// Produce the dual of an icosahedron whose faces have been subdivided according to the specified level of complexity.
    /// </summary>
    /// <param name="class1">True if making a polyhedron of Class 1 (m, 0), false if making one of Class 2 (m, m).</param>
    /// <param name="complexity">The number of new vertices added to each edge of the original shape.</param>
    /// <returns>The dual of a geodesic icosahedron of Class 1 or 2 built according to the specified level of complexity.</returns>
    public static Mesh GenerateDualGeodesicIcosahedron(bool class1, int complexity)
    {
        Mesh mesh = GenerateGeodesicPolyhedronFromIcosahedron(complexity);
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        List<Vector3> newVertices = new List<Vector3>();
        List<int> newTriangles = new List<int>();
        List<int> pentagons = new List<int>();
        List<int> hexagons = new List<int>();
        if (class1) // class 1 polyhedron (m, 0)
        {
            for (int i = 0; i < triangles.Length; i += 3) newVertices.Add((vertices[triangles[i]] + vertices[triangles[i + 1]] + vertices[triangles[i + 2]]).normalized); // add new vertex in the centre of each face of the dual shape
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    if (adjacentFaces.Count == 5) pentagons.Add(adjacentFaces[currentIndex]);
                    else if (adjacentFaces.Count == 6) hexagons.Add(adjacentFaces[currentIndex]);
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j]) // add point to current face
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
        }
        else // class 2 polyhedron (m, m)
        {
            List<int> edges = new List<int>();
            List<int> newEdges = new List<int>();
            for (int i = 0; i < vertices.Length; i++) // for each vertex of the dual shape
            {
                List<int> adjacentFaces = new List<int>();
                List<int> previousPoints = new List<int>();
                List<int> nextPoints = new List<int>();
                for (int j = 0; j < triangles.Length; j++) // for each triangle of the dual shape
                {
                    if (i == triangles[j]) // if the current triangle contains the current point
                    {
                        adjacentFaces.Add(j / 3);
                        previousPoints.Add(triangles[j % 3 == 0 ? j + 2 : j - 1]);
                        nextPoints.Add(triangles[j % 3 < 2 ? j + 1 : j - 2]);
                    }
                }
                int currentIndex = 0;
                do // find next vertex in cycle
                {
                    edges.Add(i);
                    edges.Add(nextPoints[currentIndex]);
                    newEdges.Add(newVertices.Count);
                    if (adjacentFaces.Count == 5) pentagons.Add(newVertices.Count);
                    else if (adjacentFaces.Count == 6) hexagons.Add(newVertices.Count);
                    newVertices.Add((vertices[i] + (vertices[nextPoints[currentIndex]] - vertices[i]) / 3).normalized); // add new vertex a third of the way along the edge between the two vertices and project it onto the unit sphere
                    bool foundNext = false;
                    for (int j = 0; j < adjacentFaces.Count; j++)
                    {
                        if (!foundNext && j != currentIndex && previousPoints[currentIndex] == nextPoints[j])
                        {
                            currentIndex = j;
                            foundNext = true;
                        }
                    }
                }
                while (currentIndex != 0);
            }
            for (int i = 0; i < triangles.Length; i++) // for each triangle of the dual shape
            {
                List<int> adjacentVertices = new List<int>();
                List<int> newAdjacentVertices = new List<int>();
                for (int j = 0; j < edges.Count; j += 2)
                {
                    if (triangles[i] == edges[j])
                    {
                        adjacentVertices.Add(edges[j + 1]);
                        newAdjacentVertices.Add(newEdges[j / 2]);
                    }
                }
                for (int j = 0; j < adjacentVertices.Count; j++)
                {
                    if (triangles[i % 3 < 2 ? i + 1 : i - 2] == adjacentVertices[j] && triangles[i % 3 == 0 ? i + 2 : i - 1] == adjacentVertices[(j + 1) % adjacentVertices.Count]) // add these two vertices to current hexagon
                    {
                        hexagons.Add(newAdjacentVertices[(j + 1) % newAdjacentVertices.Count]);
                        hexagons.Add(newAdjacentVertices[j]);
                    }
                }
            }
        }
        Vector3 newVertex = new Vector3(0, 0, 0);
        for (int i = 0; i < pentagons.Count; i++) // place new vertex in the centre of each pentagonal face and connect it to the edges with triangles
        {
            newVertex += newVertices[pentagons[i]] / 5;
            newTriangles.Add(pentagons[i]);
            if (i % 5 < 4)
            {
                newTriangles.Add(pentagons[i + 1]);
                newTriangles.Add(newVertices.Count);
            }
            else
            {
                newTriangles.Add(pentagons[i - 4]);
                newTriangles.Add(newVertices.Count);
                newVertices.Add(newVertex);
                newVertex = new Vector3(0, 0, 0);
            }
        }
        for (int i = 0; i < hexagons.Count; i++) // place new vertex in the centre of each hexagonal face and connect it to the edges with triangles
        {
            newVertex += newVertices[hexagons[i]] / 6;
            newTriangles.Add(hexagons[i]);
            if (i % 6 < 5)
            {
                newTriangles.Add(hexagons[i + 1]);
                newTriangles.Add(newVertices.Count);
            }
            else
            {
                newTriangles.Add(hexagons[i - 5]);
                newTriangles.Add(newVertices.Count);
                newVertices.Add(newVertex);
                newVertex = new Vector3(0, 0, 0);
            }
        }
        return new Mesh()
        {
            vertices = newVertices.ToArray(),
            triangles = newTriangles.ToArray()
        };
    }
}
