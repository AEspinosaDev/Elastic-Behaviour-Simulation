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

public class Tetrahedron
{
    public int id;

    public Node m_A;
    public Node m_B;
    public Node m_C;
    public Node m_D;

    public Tetrahedron(int id, Node a, Node b, Node c, Node d)
    {
        this.id = id;
        m_A = a;
        m_B = b;
        m_C = c;
        m_D = d;
    }
}

