using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Class representing each edge interacting with nodes and aplying compression and traction forces
/// </summary>
public class Spring
{

    public Node m_NodeA, m_NodeB;

    public float m_Length0;
    public float m_Length;
    public float m_Stiffness;

    bool m_TypeTraction;
    DeformableSolidBehaviour m_Manager;

    public Spring(Node nodeA, Node nodeB, DeformableSolidBehaviour manager, bool type)
    {
        m_NodeA = nodeA;
        m_NodeB = nodeB;
        m_Length0 = (m_NodeA.m_Pos - m_NodeB.m_Pos).magnitude;
        m_Length = m_Length0;
        m_Manager = manager;
        m_TypeTraction = type;
    }

    /// <summary>
    /// Compute de direction of the force
    /// </summary>
    public void ComputeForces()
    {

        if (m_TypeTraction) m_Stiffness = m_Manager.m_TractionStiffness;
        else m_Stiffness = m_Manager.m_FlexionStiffness;

        Vector3 u = m_NodeA.m_Pos - m_NodeB.m_Pos;
        m_Length = u.magnitude;
        u.Normalize();

        float dampForce = -m_Manager.m_SpringDamping * Vector3.Dot(u, (m_NodeA.m_Vel - m_NodeB.m_Vel));
        float stress = -m_Stiffness * (m_Length - m_Length0) + dampForce;
        Vector3 force = stress * u;
        m_NodeA.m_Force += force;
        m_NodeB.m_Force -= force;

    }
}



//----------------------------------------------
//    AUXILIAR CLASSES
//----------------------------------------------



/// <summary>
/// Auxiliar class to temporary compare the edges in the mesh looking for repeated ones.
/// </summary>
public class Edge
{
    public int m_A, m_B, m_O;

    public Edge(int a, int b, int o)
    {
        if (a < b)
        {
            m_A = a;
            m_B = b;

        }
        else
        {
            m_B = a;
            m_A = b;
        }
        m_O = o;
    }
}
/// <summary>
/// Custom comparer class designed for the Edge class
/// </summary>
public class EdgeQualityComparer : IEqualityComparer<Edge>
{
    public bool Equals(Edge a, Edge b)
    {
        if (a.m_A == b.m_A && a.m_B == b.m_B || a.m_A == b.m_B && a.m_B == b.m_A) return true;
        else return false;
    }

    public int GetHashCode(Edge e)
    {
        List<int> pts = new List<int>(); pts.Add(e.m_A); pts.Add(e.m_B);
        pts.Sort();
        //CANTOR PAIRING FUNCTION
        int hcode = ((pts[0] + pts[1]) * (pts[0] + pts[1] + 1)) / 2 + pts[1];

        return hcode.GetHashCode();
    }
}
/// <summary>
/// Class that represents the Tetahedron, the poligons which the proxy mesh is made of.
/// </summary>
public class Tetrahedron
{
    public int id;

    public Node m_A;
    public Node m_B;
    public Node m_C;
    public Node m_D;

    public float m_Volume;

    public Tetrahedron(int id, Node a, Node b, Node c, Node d)
    {
        this.id = id;
        m_A = a;
        m_B = b;
        m_C = c;
        m_D = d;

        m_Volume = ComputeVolume();
        Debug.Log("Volume" + m_Volume);
    }
    private float ComputeVolume()
    {

        Vector3 crossProduct = Vector3.Cross(m_A.m_Pos - m_D.m_Pos, m_B.m_Pos - m_D.m_Pos);
        return Mathf.Abs(Vector3.Dot(crossProduct, m_C.m_Pos - m_D.m_Pos)) / 6;
    }

    public void ComputeVertexWeights(Vector3 p, out float wA, out float wB, out float wC, out float wD)
    {
        Vector3 vP_A = m_A.m_Pos - p;
        Vector3 vP_B = m_B.m_Pos - p;
        Vector3 vP_C = m_C.m_Pos - p;
        Vector3 vP_D = m_D.m_Pos - p;
        
        //Debug.Log(m_A.m_Pos + " " + m_B.m_Pos + " " + m_C.m_Pos + " "+m_D.m_Pos);
        //Debug.Log(p);
        //wA
        Vector3 crossProduct = Vector3.Cross(vP_B, vP_C);
        float vA = (Mathf.Abs(Vector3.Dot(crossProduct, vP_D)) / 6);
        wA = vA / m_Volume;

        //wB
        crossProduct = Vector3.Cross(vP_A, vP_C);
        float vB = (Mathf.Abs(Vector3.Dot(crossProduct, vP_D)) / 6);
        wB = vB / m_Volume;

        //wC
        crossProduct = Vector3.Cross(vP_A, vP_B);
        float vC = (Mathf.Abs(Vector3.Dot(crossProduct, vP_D)) / 6);
        wC = vC / m_Volume;

        //wD
        crossProduct = Vector3.Cross(vP_A, vP_B);
        float vD = (Mathf.Abs(Vector3.Dot(crossProduct, vP_C)) / 6);
        wD = vD / m_Volume;

        //Debug.Log(vA + " " + vB + " " + vC + " " + vD);
    }

    private bool SameSide(Vector3 p1,Vector3 p2, Vector3 p3, Vector3 p4,Vector3 point)
    {

        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1);
        float p4dist = Vector3.Dot(normal, p4 - p1);
        float pdist = Vector3.Dot(normal, point - p1);
        return Mathf.Sign(p4dist) <= Mathf.Sign(pdist);
    }


    public bool PointInside(Vector3 p)
    {
        Vector3 normal2 = Vector3.Cross(m_C.m_Pos - m_A.m_Pos, m_D.m_Pos - m_A.m_Pos);
        Vector3 normal1 = Vector3.Cross(m_B.m_Pos - m_A.m_Pos, m_C.m_Pos - m_A.m_Pos);
        Vector3 normal3 = Vector3.Cross(m_D.m_Pos - m_A.m_Pos, m_B.m_Pos - m_A.m_Pos);
        Vector3 normal4 = Vector3.Cross(m_D.m_Pos - m_B.m_Pos, m_C.m_Pos - m_B.m_Pos);

        //Vector3 dir1 = m_A.m_Pos - p;
        //Vector3 dir2 = m_D.m_Pos - p;


        //Debug.Log(Mathf.Cos(Vector3.Angle(normal1, dir1)));
        //Debug.Log(Mathf.Cos(Vector3.Angle(normal2, dir1)));
        //Debug.Log(Mathf.Cos(Vector3.Angle(normal3, dir1)));
        //Debug.Log(Mathf.Cos(Vector3.Angle(normal4, dir2)));
        //if (Mathf.Cos(Vector3.Angle(normal1, dir1)) >= -1 &&
        //    Mathf.Cos(Vector3.Angle(normal2, dir1)) >= -1 &&
        //    Mathf.Cos(Vector3.Angle(normal3, dir1)) >= -1 &&
        //    Mathf.Cos(Vector3.Angle(normal4, dir2)) >= -1) return true;


        return normal1.x * (m_A.m_Pos.x - p.x) + normal1.y * (m_A.m_Pos.y - p.y) + normal1.z * (m_A.m_Pos.z - p.z) <= 0 &&
            normal2.x * (m_A.m_Pos.x - p.x) + normal2.y * (m_A.m_Pos.y - p.y) + normal2.z * (m_A.m_Pos.z - p.z) <= 0 &&
            normal3.x * (m_A.m_Pos.x - p.x) + normal3.y * (m_A.m_Pos.y - p.y) + normal3.z * (m_A.m_Pos.z - p.z) <= 0 &&
            normal4.x * (m_D.m_Pos.x - p.x) + normal4.y * (m_D.m_Pos.y - p.y) + normal4.z * (m_D.m_Pos.z - p.z) <= 0; 

        //return SameSide(m_A.m_Pos, m_B.m_Pos, m_C.m_Pos, m_D.m_Pos, p) &&
        //    SameSide(m_B.m_Pos, m_C.m_Pos, m_D.m_Pos, m_A.m_Pos, p) &&
        //    SameSide(m_C.m_Pos, m_D.m_Pos, m_A.m_Pos, m_B.m_Pos, p) &&
        //    SameSide(m_D.m_Pos, m_A.m_Pos, m_B.m_Pos, m_C.m_Pos, p);

        //return false;
    }

}

